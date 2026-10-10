/**
 * @file bq796xx_bms_monitor.c
 * @brief Aquisicao transaccional, proteccoes, falhas e balanceamento.
 * Dados "raw" significam antes do IIR de software; o LPF interno do BQ
 * continua activo. Esta camada NAO escreve nos pinos dos reles.
 */
#include "bq796xx_bms.h"
#include "bms_relays.h"

/**
 * @brief Reconstroi uma palavra de 16 bits a partir de HI/LO.
 *
 * Para que serve: converter a ordem dos bytes dos resultados ADC/registos.
 *
 * Nao aplica escala nem valida o codigo ADC; o chamador faz esses passos.
 */
static uint16_t word_be(const uint8_t *p)
{
    return (uint16_t)(((uint16_t)p[0] << 8U) | p[1]);
}

/**
 * @brief Traduz um resultado de API numa categoria de falha.
 *
 * Para que serve: preservar a diferenca entre timeout/comunicacao, CRC, frame e plataforma.
 *
 * Funcao interna usada em caminhos de erro; nao representa a leitura de FAULT_SUMMARY.
 */
static uint32_t fault_for_status(BMS_Status_t st)
{
    return BMS_FaultForStatus(st);
}

/**
 * @brief Retira permissoes logicas sem fazer comunicacao bloqueante.
 *
 * Para que serve: reagir de imediato a uma falha mesmo antes de conseguir ler os BQ.
 *
 * Como funciona: marca safety_latched e coloca as permissoes globais a falso.
 *
 * Pode ser usada na ISR. Nao escreve nos pinos de reles.
 */
void BMS_InhibitOutputs(BMS_Handle_t *h)
{
    if (h == NULL)
    {
        return;
    }
    __atomic_store_n(&h->safety_latched, 1U, __ATOMIC_SEQ_CST);
    h->bms_ok = false;
    h->contactor_closed = false;
    h->precharge_ready = false;
}

/**
 * @brief Passa a FAULT e pede paragem de balanceamento quando necessaria.
 *
 * Para que serve: nao manter um estado MONITORING depois de uma falha detectada.
 *
 * Como funciona: inibe imediatamente e agenda a ordem BQ de paragem fora da ISR.
 *
 * Apesar do nome, nao envia aqui um ping SHUTDOWN nem espera pela UART.
 */
void BMS_EmergencyShutdown(BMS_Handle_t *h)
{
    if (h == NULL)
    {
        return;
    }
    /* Nao esperar pela UART para retirar permissoes. A paragem do
     * balanceamento e tentada pela tarefa principal, fora da ISR. */
    BMS_InhibitOutputs(h);
    h->state = BMS_STATE_FAULT;
    h->balance_stop_pending = h->balance_stop_pending || h->is_balancing;
    h->physical_safe_confirmed = false;
}

/**
 * @brief Consulta NFAULT por nivel fisico, EXTI e evento pendente.
 *
 * Para que serve: detectar tambem um pino que ja estava LOW antes de surgir um novo flanco.
 *
 * Devolve true para handle nulo. Nao limpa a interrupcao nem os registos de falha.
 */
bool BMS_NfaultIsActive(const BMS_Handle_t *h)
{
    return h == NULL || HAL_GPIO_ReadPin(BMS_NFAULT_PORT, BMS_NFAULT_PIN) == GPIO_PIN_RESET ||
           __HAL_GPIO_EXTI_GET_IT(BMS_NFAULT_PIN) != RESET ||
           __atomic_load_n(&h->nfault_pending, __ATOMIC_SEQ_CST) != 0U;
}

/**
 * @brief Verifica se as amostras existem e ainda sao recentes.
 *
 * Para que serve: distinguir o ultimo numero guardado de uma medicao utilizavel agora.
 *
 * Como funciona: verifica flags e timestamps globais e por slave; tensoes sao
 * sempre exigidas. Temperaturas/HV so se o chamador as pedir.
 *
 * Nao comunica com os BQ e nao renova a idade das amostras.
 */
bool BMS_MeasurementsFresh(const BMS_Handle_t *h, bool require_t, bool require_hv)
{
    if (h == NULL || !h->voltage_valid)
    {
        return false;
    }
    uint32_t now = HAL_GetTick();
    if ((uint32_t)(now - h->last_voltage_ms) > BMS_MEASUREMENT_MAX_AGE_MS)
    {
        return false;
    }
    for (uint8_t s = 0U; s < BMS_NUM_SLAVES; ++s)
    {
        if (!h->slave[s].voltage_valid ||
            (uint32_t)(now - h->slave[s].last_voltage_ms) > BMS_MEASUREMENT_MAX_AGE_MS)
        {
            return false;
        }
        if (require_t &&
            (!h->slave[s].temperature_valid ||
             (uint32_t)(now - h->slave[s].last_temperature_ms) > BMS_MEASUREMENT_MAX_AGE_MS))
        {
            return false;
        }
    }
    if (require_t && (!h->temperature_valid ||
                      (uint32_t)(now - h->last_temperature_ms) > BMS_MEASUREMENT_MAX_AGE_MS))
    {
        return false;
    }
    if (require_hv && (!h->inverter_voltage_valid ||
                       (uint32_t)(now - h->last_inverter_ms) > BMS_MEASUREMENT_MAX_AGE_MS))
    {
        return false;
    }
    return true;
}

/**
 * @brief Verifica se o driver pode pedir resultados de ADC.
 *
 * Para que serve: impedir leituras de uma rede nao enderecada ou desconfigurada.
 *
 * Permite ler durante o diagnostico de FAULT; ler nao rearma o sistema.
 */
static bool acquisition_ready(const BMS_Handle_t *h)
{
    return h != NULL && h->addresses_valid && h->direction_valid && h->config_verified &&
           h->state != BMS_STATE_SHUTDOWN && h->state != BMS_STATE_SLEEP &&
           h->state != BMS_STATE_UNINITIALIZED;
}

/**
 * @brief Adquire as 30 tensoes e publica um conjunto completo.
 *
 * Para que serve: obter C1..C15 de cada slave, a soma do pack e min/max/delta.
 *
 * Como funciona: 1) le 30 bytes desde VCELL15_HI; 2) inverte a ordem de canais;
 * 3) rejeita codigos invalidos/negativos; 4) converte a mV; 5) publica so
 * depois de ambos os slaves responderem correctamente.
 *
 * A resolucao e 190.73 uV/LSB; usa inteiros e arredonda no fim.
 * Uma falha conserva o historico para debug, mas marca-o como nao valido.
 */
BMS_Status_t BMS_ReadAllCellVoltages(BMS_Handle_t *h)
{
    if (h == NULL)
    {
        return BMS_ERR_INVALID_PARAM;
    }
    h->voltage_valid = false;
    h->precharge_ready = false;
    if (!acquisition_ready(h))
    {
        BMS_InhibitOutputs(h);
        return BMS_RecordError(
            h, BMS_ERR_NOT_READY, "ACQUISITION_NOT_READY", 0xFFU, REG_VCELL15_HI, 1U, 0U, 0U);
    }
    /* Area temporaria: nenhum canal novo e publicado se o outro slave falhar. */
    uint16_t next[BMS_NUM_SLAVES][BMS_CELLS_PER_SLAVE];
    uint8_t data[BMS_VCELL_READ_BYTES];
    BMS_Status_t result = BMS_OK;
    uint32_t sum = 0U;
    uint16_t minimum = UINT16_MAX, maximum = 0U;
    for (uint8_t s = 0U; s < BMS_NUM_SLAVES; ++s)
    {
        h->slave[s].voltage_valid = false;
        BMS_Status_t st = BMS_ReadSingleDevice(
            h, BMS_GetSlaveAddress(h, s), REG_VCELL15_HI, data, BMS_VCELL_READ_BYTES);
        if (st != BMS_OK)
        {
            h->slave[s].comm_ok = false;
            h->slave[s].fault_flags |= fault_for_status(st);
            h->fault_flags |= fault_for_status(st);
            result = st;
            continue;
        }
        h->slave[s].comm_ok = true;
        for (uint8_t c = 0U; c < BMS_CELLS_PER_SLAVE; ++c)
        {
            /* Os registos chegam de C15 para C1; o array publico usa C1 no indice zero. */
            uint16_t raw = word_be(&data[(BMS_CELLS_PER_SLAVE - 1U - c) * 2U]);
            /* Formato complemento para dois. 0x8000 e invalido/inactivo.
             * Uma tensao negativa nao e convertida num enorme unsigned. */
            if (raw >= 0x8000U)
            {
                h->fault_flags |= BMS_FAULT_SENSOR;
                h->slave[s].fault_flags |= BMS_FAULT_SENSOR;
                (void)BMS_RecordError(h,
                                      BMS_ERR_FAULT_ACTIVE,
                                      "CELL_ADC_INVALID",
                                      BMS_GetSlaveAddress(h, s),
                                      (uint16_t)(REG_VCELL1_HI - 2U * c),
                                      0U,
                                      raw,
                                      0x8000U);
                result = BMS_ERR_FAULT_ACTIVE;
                break;
            }
            /* Escala em centesimos de microvolt; +50000 arredonda a divisao para milivolts. */
            uint16_t mv = (uint16_t)(((uint32_t)raw * BMS_CELL_LSB_X100_UV + 50000UL) / 100000UL);
            next[s][c] = mv;
            sum += mv;
            if (mv < minimum)
            {
                minimum = mv;
            }
            if (mv > maximum)
            {
                maximum = mv;
            }
        }
    }
    if (result != BMS_OK)
    {
        /* As amostras anteriores ficam disponiveis para diagnostico, mas
         * NAO se publica uma soma de apenas metade do pack. */
        h->filter_primed = false;
        h->soc_valid = false;
        BMS_EmergencyShutdown(h);
        return result;
    }
    uint32_t now = HAL_GetTick();
    for (uint8_t s = 0U; s < BMS_NUM_SLAVES; ++s)
    {
        memcpy(h->slave[s].cell_voltage_mv, next[s], sizeof(next[s]));
        h->slave[s].voltage_valid = true;
        h->slave[s].last_voltage_ms = now;
    }
    h->pack_voltage_mv = sum;
    h->min_cell_mv = minimum;
    h->max_cell_mv = maximum;
    h->delta_cell_mv = (uint16_t)(maximum - minimum);
    h->last_voltage_ms = now;
    h->voltage_valid = true;
    return BMS_OK;
}

typedef struct
{
    uint16_t ratio_x10000;
    int8_t temp_c;
} NTC_Point_t;

/* Tabela do projecto mantida: NTC 10 kohm, beta=3950, pull-up 10 kohm.
 * Validar componentes e tolerancias reais. Nao e uma calibracao. */
static const NTC_Point_t g_ntc_table[BMS_NTC_TABLE_SIZE] = {
    /*  ratio_x10000   temp_°C       R_NTC (Ω)    */
    {1127U, 80},  /* R_NTC=     1270       */
    {1298U, 75},  /* R_NTC=     1492       */
    {1496U, 70},  /* R_NTC=     1760       */
    {1726U, 65},  /* R_NTC=     2086       */
    {1991U, 60},  /* R_NTC=     2486       */
    {2295U, 55},  /* R_NTC=     2978       */
    {2641U, 50},  /* R_NTC=     3588       */
    {3030U, 45},  /* R_NTC=     4348       */
    {3465U, 40},  /* R_NTC=     5301       */
    {3941U, 35},  /* R_NTC=     6506       */
    {4456U, 30},  /* R_NTC=     8037       */
    {5000U, 25},  /* R_NTC=    10000  <- ponto nominal */
    {5563U, 20},  /* R_NTC=    12535       */
    {6130U, 15},  /* R_NTC=    15837       */
    {6686U, 10},  /* R_NTC=    20175       */
    {7216U, 5},   /* R_NTC=    25925       */
    {7708U, 0},   /* R_NTC=    33621       */
    {8149U, -5},  /* R_NTC=    44026       */
    {8535U, -10}, /* R_NTC=    58246       */
    {8862U, -15}, /* R_NTC=    77898       */
    {9133U, -20}, /* R_NTC=   105385       */
};

/**
 * @brief Converte os codigos ADC de GPIO/TSREF numa temperatura inteira em graus C.
 *
 * Valida os codigos, a referencia TSREF e a razao do divisor NTC. Compensa as
 * escalas diferentes dos dois ADC antes de interpolar a tabela do projecto.
 * Inicializa a saida como invalida; curto, circuito aberto ou valores fora da
 * gama suportada devolvem falha em vez de uma temperatura plausivel inventada.
 *
 * @param gpio Codigo ADC do canal ligado ao divisor NTC.
 * @param tsref Codigo ADC da referencia de excitacao do divisor.
 * @param temperature Destino da temperatura em graus C; obrigatorio.
 * @return BMS_OK apenas se a conversao foi aceite. A tabela nao substitui calibracao.
 */
static BMS_Status_t raw_temperature(uint16_t gpio, uint16_t tsref, int16_t *temperature)
{
    if (temperature == NULL)
    {
        return BMS_ERR_INVALID_PARAM;
    }
    *temperature = BMS_INVALID_TEMPERATURE_C;
    if (gpio >= BMS_ADC_INVALID_CODE || tsref == 0U || tsref >= BMS_ADC_INVALID_CODE)
    {
        return BMS_ERR_FAULT_ACTIVE;
    }
    uint32_t ref_mv = ((uint32_t)tsref * BMS_TSREF_LSB_X100_UV + 50000UL) / 100000UL;
    if (ref_mv < BMS_TSREF_MIN_MV || ref_mv > BMS_TSREF_MAX_MV)
    {
        return BMS_ERR_FAULT_ACTIVE;
    }
    /* Razao V_GPIO/V_TSREF multiplicada por 10000, corrigindo as escalas de ambos os canais. */
    uint32_t ratio = (uint32_t)(((uint64_t)gpio * BMS_GPIO_LSB_X100_UV * 10000UL) /
                                ((uint64_t)tsref * BMS_TSREF_LSB_X100_UV));
    if (ratio < BMS_NTC_RATIO_MIN || ratio > BMS_NTC_RATIO_MAX)
    {
        return BMS_ERR_FAULT_ACTIVE;
    } /* Sensor curto/aberto, nao "frio". */
    if (ratio <= g_ntc_table[0].ratio_x10000)
    {
        *temperature = g_ntc_table[0].temp_c;
        return BMS_OK;
    } /* >=80 C, ja excede OT. */
    if (ratio > g_ntc_table[BMS_NTC_TABLE_SIZE - 1U].ratio_x10000)
    {
        return BMS_ERR_FAULT_ACTIVE;
    } /* Fora da tabela: nao inventar temperatura. */
    for (uint8_t i = 1U; i < BMS_NTC_TABLE_SIZE; ++i)
    {
        if (ratio <= g_ntc_table[i].ratio_x10000)
        {
            int32_t dr = (int32_t)(ratio - g_ntc_table[i - 1U].ratio_x10000);
            int32_t span =
                (int32_t)(g_ntc_table[i].ratio_x10000 - g_ntc_table[i - 1U].ratio_x10000);
            int32_t dt = (int32_t)g_ntc_table[i].temp_c - g_ntc_table[i - 1U].temp_c;
            /* Interpolar primeiro em centesimos: evita um salto de 1 grau
             * por truncar a parcela negativa antes de somar a origem. */
            int32_t centi = (int32_t)g_ntc_table[i - 1U].temp_c * 100 + dt * dr * 100 / span;
            *temperature = (int16_t)((centi >= 0 ? centi + 50 : centi - 50) / 100);
            return BMS_OK;
        }
    }
    return BMS_ERR_FAULT_ACTIVE;
}

/**
 * @brief Le e converte os canais NTC dos dois slaves.
 *
 * Le TSREF e GPIOs, converte cada canal com raw_temperature e calcula a maxima.
 * So publica o conjunto quando todas as leituras/conversoes foram aceites;
 * em erro invalida a temperatura global e entra em FAULT, preservando a causa.
 * Esta API existe, mas a aplicacao de bancada mantem a sua chamada comentada.
 */
BMS_Status_t BMS_ReadAllTemperatures(BMS_Handle_t *h)
{
    if (h == NULL)
    {
        return BMS_ERR_INVALID_PARAM;
    }
    h->temperature_valid = false;
    if (!acquisition_ready(h))
    {
        BMS_InhibitOutputs(h);
        return BMS_RecordError(
            h, BMS_ERR_NOT_READY, "TEMPERATURE_NOT_READY", 0xFFU, REG_TSREF_HI, 1U, 0U, 0U);
    }
    /* Tal como nas tensoes, os resultados so se tornam validos como conjunto completo. */
    int16_t next[BMS_NUM_SLAVES][BMS_NUM_TEMP_SENSORS], maximum = INT16_MIN;
    uint8_t buf[BMS_AUX_READ_BYTES];
    BMS_Status_t result = BMS_OK;
    for (uint8_t s = 0U; s < BMS_NUM_SLAVES; ++s)
    {
        h->slave[s].temperature_valid = false;
        BMS_Status_t st =
            BMS_ReadSingleDevice(h, BMS_GetSlaveAddress(h, s), REG_TSREF_HI, buf, sizeof(buf));
        if (st != BMS_OK)
        {
            result = st;
            h->fault_flags |= fault_for_status(st);
            continue;
        }
        uint16_t tsref = word_be(buf);
        for (uint8_t c = 0U; c < BMS_NUM_TEMP_SENSORS; ++c)
        {
            st = raw_temperature(word_be(&buf[2U + 2U * c]), tsref, &next[s][c]);
            if (st != BMS_OK)
            {
                h->fault_flags |= BMS_FAULT_SENSOR;
                h->slave[s].fault_flags |= BMS_FAULT_SENSOR;
                result = st;
                (void)BMS_RecordError(h,
                                      st,
                                      "TEMPERATURE_ADC_INVALID",
                                      BMS_GetSlaveAddress(h, s),
                                      (uint16_t)(REG_TSREF_HI + 2U + 2U * c),
                                      0U,
                                      ((uint32_t)tsref << 16U) | word_be(&buf[2U + 2U * c]),
                                      0U);
            }
            else if (next[s][c] > maximum)
            {
                maximum = next[s][c];
            }
        }
    }
    if (result != BMS_OK)
    {
        h->max_temp_c = BMS_INVALID_TEMPERATURE_C;
        BMS_EmergencyShutdown(h);
        return result;
    }
    uint32_t now = HAL_GetTick();
    for (uint8_t s = 0U; s < BMS_NUM_SLAVES; ++s)
    {
        memcpy(h->slave[s].temperatures_c, next[s], sizeof(next[s]));
        h->slave[s].temperature_valid = true;
        h->slave[s].last_temperature_ms = now;
    }
    h->max_temp_c = maximum;
    h->last_temperature_ms = now;
    h->temperature_valid = true;
    return BMS_OK;
}

/**
 * @brief Mede a tensao do bus do inversor a partir do GPIO4 do primeiro slave.
 *
 * Valida a palavra ADC, aplica a escala do GPIO e a atenuacao do sensor HV
 * usando aritmetica de 64 bits. Publica milivolts e timestamp apenas se o valor
 * for plausivel; em erro invalida HV e a permissao de pre-carga e entra em FAULT.
 * A aquisicao depende do sensor/divisor real; a chamada esta desactivada em bancada.
 */
BMS_Status_t BMS_ReadInverterVoltage(BMS_Handle_t *h)
{
    if (h == NULL)
    {
        return BMS_ERR_INVALID_PARAM;
    }
    h->inverter_voltage_valid = false;
    h->precharge_ready = false;
    h->inverter_voltage_mv = 0U;
    if (!acquisition_ready(h))
    {
        BMS_InhibitOutputs(h);
        return BMS_RecordError(
            h, BMS_ERR_NOT_READY, "HV_NOT_READY", 0xFFU, REG_GPIO4_HI, 1U, 0U, 0U);
    }
    uint8_t data[2U];
    BMS_Status_t st = BMS_ReadSingleDevice(h, BMS_GetSlaveAddress(h, 0U), REG_GPIO4_HI, data, 2U);
    if (st != BMS_OK)
    {
        h->fault_flags |= fault_for_status(st);
        BMS_EmergencyShutdown(h);
        return st;
    }
    uint16_t raw = word_be(data);
    if (raw >= BMS_ADC_INVALID_CODE)
    {
        h->fault_flags |= BMS_FAULT_SENSOR;
        BMS_EmergencyShutdown(h);
        return BMS_RecordError(h,
                               BMS_ERR_FAULT_ACTIVE,
                               "HV_ADC_INVALID",
                               BMS_GetSlaveAddress(h, 0U),
                               REG_GPIO4_HI,
                               0U,
                               raw,
                               0x8000U);
    }
    /* 152.59 uV/LSB, NAO a escala de VCELL. Arredondar so no fim. */
    uint64_t numerator = (uint64_t)raw * BMS_GPIO_LSB_X100_UV * HV_BUS_ATTENUATION_NUM;
    uint64_t denominator = 100000ULL * HV_BUS_ATTENUATION_DEN;
    uint32_t mv = (uint32_t)((numerator + denominator / 2U) / denominator);
    if (mv > BMS_HV_MAX_PLAUSIBLE_MV)
    {
        h->fault_flags |= BMS_FAULT_SENSOR;
        BMS_EmergencyShutdown(h);
        return BMS_RecordError(h,
                               BMS_ERR_FAULT_ACTIVE,
                               "HV_IMPLAUSIBLE",
                               BMS_GetSlaveAddress(h, 0U),
                               REG_GPIO4_HI,
                               BMS_HV_MAX_PLAUSIBLE_MV,
                               mv,
                               0U);
    }
    h->inverter_voltage_mv = mv;
    h->last_inverter_ms = HAL_GetTick();
    h->inverter_voltage_valid = true;
    BMS_UpdateHardwareInterlocks(h);
    return BMS_OK;
}

/**
 * @brief Compara as amostras com os limites de software existentes.
 *
 * Para que serve: assinalar OV/UV mesmo alem da verificacao dos comparadores BQ.
 *
 * Como funciona: verifica idade; percorre todas as celulas; acumula falhas
 * latched e avisos. Usa amostras anteriores ao IIR de software.
 *
 * Em bancada, as chamadas NTC/HV desactivadas nao impedem o ensaio de
 * tensoes; continuam sem permitir balanceamento ou autorizacao global.
 */
BMS_Status_t BMS_CheckProtections(BMS_Handle_t *h)
{
    if (h == NULL)
    {
        return BMS_ERR_INVALID_PARAM;
    }
    bool operational = (BMS_BENCH_MODE == 0U);
    if (!BMS_MeasurementsFresh(h, operational, false))
    {
        h->fault_flags |= BMS_FAULT_STALE_DATA;
        BMS_EmergencyShutdown(h);
        return BMS_RecordError(h,
                               BMS_ERR_NOT_READY,
                               "MEASUREMENTS_NOT_FRESH",
                               0xFFU,
                               REG_VCELL15_HI,
                               BMS_MEASUREMENT_MAX_AGE_MS,
                               (uint32_t)(HAL_GetTick() - h->last_voltage_ms),
                               0U);
    }
    /* Avisos sao recalculados; o aviso de anel degradado persiste. Falhas ficam retidas em fault_flags. */
    uint32_t warnings = h->warning_flags & BMS_WARN_RING_DEGRADED;
    if (!h->temperature_valid)
    {
        warnings |= BMS_WARN_TEMP_UNAVAILABLE;
    }
    if (!h->inverter_voltage_valid)
    {
        warnings |= BMS_WARN_HV_UNAVAILABLE;
    }
    for (uint8_t s = 0U; s < BMS_NUM_SLAVES; ++s)
    {
        for (uint8_t c = 0U; c < BMS_CELLS_PER_SLAVE; ++c)
        {
            uint16_t mv = h->slave[s].cell_voltage_mv[c];
            /* Comparar amostras anteriores ao IIR evita atrasar a proteccao pelo filtro de visualizacao. */
            bool ov = mv >= CELL_OV_MV, uv = mv <= CELL_UV_MV;
            h->slave[s].ov_cell[c] = h->slave[s].ov_cell[c] || ov;
            h->slave[s].uv_cell[c] = h->slave[s].uv_cell[c] || uv;
            if (ov)
            {
                h->fault_flags |= BMS_FAULT_OV;
                h->slave[s].fault_flags |= BMS_FAULT_OV;
            }
            if (uv)
            {
                h->fault_flags |= BMS_FAULT_UV;
                h->slave[s].fault_flags |= BMS_FAULT_UV;
            }
            if (ov || uv)
            {
                (void)BMS_RecordError(h,
                                      BMS_ERR_FAULT_ACTIVE,
                                      ov ? "CELL_OV_MV" : "CELL_UV_MV",
                                      BMS_GetSlaveAddress(h, s),
                                      (uint16_t)(REG_VCELL1_HI - 2U * c),
                                      ov ? CELL_OV_MV : CELL_UV_MV,
                                      mv,
                                      0U);
            }
            if (mv >= CELL_WARN_OV_MV)
            {
                warnings |= BMS_WARN_OV;
            }
            if (mv <= CELL_WARN_UV_MV)
            {
                warnings |= BMS_WARN_UV;
            }
        }
        if (h->slave[s].temperature_valid)
        {
            for (uint8_t t = 0U; t < BMS_NUM_TEMP_SENSORS; ++t)
            {
                int16_t temp = h->slave[s].temperatures_c[t];
                if (temp == BMS_INVALID_TEMPERATURE_C)
                {
                    h->fault_flags |= BMS_FAULT_SENSOR;
                }
                else if (temp >= (int16_t)CELL_TEMP_MAX_C)
                {
                    h->fault_flags |= BMS_FAULT_OT;
                    h->slave[s].fault_flags |= BMS_FAULT_OT;
                }
                if (temp != BMS_INVALID_TEMPERATURE_C && temp >= (int16_t)CELL_TEMP_WARN_C)
                {
                    warnings |= BMS_WARN_TEMP;
                }
            }
        }
    }
    if (h->delta_cell_mv > CELL_IMBALANCE_MV)
    {
        warnings |= BMS_WARN_IMBALANCE;
    }
    h->warning_flags = warnings;
    if (h->fault_flags != 0U)
    {
        BMS_EmergencyShutdown(h);
        return BMS_ERR_FAULT_ACTIVE;
    }
    return BMS_OK;
}

/**
 * @brief Regista um evento NFAULT sem efectuar UART.
 *
 * Para que serve: inibir rapidamente e delegar o diagnostico ao contexto principal.
 *
 * Como funciona: actualiza o contador de eventos e a flag pendente atomicamente.
 *
 * Nao confundir o evento com a causa: a causa e lida em FAULT_SUMMARY/detalhes.
 */
void BMS_NFAULT_IRQHandler(BMS_Handle_t *h)
{
    if (h == NULL)
    {
        return;
    }
    BMS_InhibitOutputs(h);
    __atomic_add_fetch(&h->nfault_epoch, 1U, __ATOMIC_SEQ_CST);
    __atomic_store_n(&h->nfault_pending, 1U, __ATOMIC_SEQ_CST);
    __atomic_add_fetch(&h->fault_count, 1U, __ATOMIC_SEQ_CST);
}

/* Le as categorias completas. Uma falha desconhecida nao se transforma em OK.
 * Leitura de VCOW nao equivale a executar o diagnostico de open-wire. */
/**
 * @brief Le e interpreta falhas da bridge e dos dois slaves.
 *
 * Para que serve: manter separadas as causas de comunicacao, OV/UV e hardware.
 *
 * Como funciona: le o resumo/detalhes da bridge, depois resumo dos slaves e os
 * detalhes necessarios. Guarda tambem bytes raw para o depurador.
 *
 * BMS_OK significa que a consulta terminou; *faults pode continuar diferente
 * de zero. Ler VCOW nao significa que o teste open-wire foi executado.
 */
BMS_Status_t BMS_ScanHardwareFaults(BMS_Handle_t *h, uint32_t *faults)
{
    if (h == NULL || faults == NULL)
    {
        return BMS_ERR_INVALID_PARAM;
    }
    /* Reiniciar a validade do snapshot. Uma leitura interrompida nunca pode parecer um scan completo. */
    *faults = 0U;
    h->fault_scan_valid = false;
    h->bridge_fault_valid = false;
    for (uint8_t s = 0U; s < BMS_NUM_SLAVES; ++s)
    {
        BMS_SlaveData_t *sl = &h->slave[s];
        memset(sl->fault_summary_raw, 0, sizeof(sl->fault_summary_raw));
        memset(sl->fault_detail_raw, 0, sizeof(sl->fault_detail_raw));
        memset(sl->fault_comp_raw, 0, sizeof(sl->fault_comp_raw));
        memset(sl->fault_ovuv_raw, 0, sizeof(sl->fault_ovuv_raw));
        memset(sl->fault_otut_raw, 0, sizeof(sl->fault_otut_raw));
        sl->fault_detail_valid = 0U;
        sl->fault_comp_valid = 0U;
        sl->fault_summary_valid = sl->fault_comm_valid = false;
        sl->fault_ovuv_valid = sl->fault_otut_valid = false;
        /* init_pwr_raw e historico do arranque: nao apagar depois de RST_PWR. */
    }
    if (!h->addresses_valid || !h->direction_valid)
    {
        *faults = BMS_FAULT_COMM;
        return BMS_ERR_NOT_READY;
    }
    uint8_t bridge[6U], summary[BMS_NUM_SLAVES], buf[4U];
    BMS_Status_t st =
        BMS_ReadSingleDevice(h, BMS_ADDR_BRIDGE, REG_BRIDGE_FAULT_SUMMARY, bridge, sizeof(bridge));
    if (st != BMS_OK)
    {
        *faults |= fault_for_status(st);
        return st;
    }
    memcpy(h->bridge_fault_raw, bridge, sizeof(bridge));
    h->bridge_fault_valid = true;
    /* SYS e COMM sao classificados pelo detalhe. REG/PWR e bits de resumo
     * reservados continuam HARDWARE, mesmo que o detalhe esteja a zero. */
    if ((bridge[0] & (uint8_t)~(BRIDGE_SUMMARY_SYS | BRIDGE_SUMMARY_COMM)) != 0U ||
        bridge[1] != 0U || bridge[3] != 0U)
    {
        *faults |= BMS_FAULT_HARDWARE;
    }
    if ((bridge[2] & BRIDGE_SYS_COMM_TIMEOUTS) != 0U)
    {
        *faults |= BMS_FAULT_COMM;
    }
    if ((bridge[2] & (uint8_t)~BRIDGE_SYS_COMM_TIMEOUTS) != 0U ||
        ((bridge[0] & BRIDGE_SUMMARY_SYS) != 0U && bridge[2] == 0U))
    {
        *faults |= BMS_FAULT_HARDWARE;
    }
    /* COMMCLR_DET/STOP_DET nao sao filtrados: tambem continuam COMM. */
    if (bridge[4] != 0U || bridge[5] != 0U || (bridge[0] & BRIDGE_SUMMARY_COMM) != 0U)
    {
        *faults |= BMS_FAULT_COMM;
    }
    if ((bridge[4] & BRIDGE_COMM1_HB_ERRORS) != 0U)
    {
        *faults |= BMS_FAULT_HB_FAIL;
    }
    if ((bridge[5] & BRIDGE_COMM2_LINK_ERR) != 0U)
    {
        h->ring_intact = false;
        *faults |= BMS_FAULT_RING_BREAK;
    }
    if (*faults != 0U)
    {
        (void)BMS_RecordError(h,
                              BMS_ERR_FAULT_ACTIVE,
                              "BRIDGE_FAULT",
                              BMS_ADDR_BRIDGE,
                              REG_BRIDGE_FAULT_SUMMARY,
                              0U,
                              bridge[0],
                              0xFFU);
    }
    st = BMS_ReadStack(h, REG_SLAVE_FAULT_SUMMARY, summary, 1U);
    if (st != BMS_OK)
    {
        *faults |= fault_for_status(st);
        return st;
    }
    /* A leitura Stack ja obteve ambos os resumos. Conservar tambem o de S2
     * se uma leitura de detalhe de S1 falhar a seguir. */
    for (uint8_t s = 0U; s < BMS_NUM_SLAVES; ++s)
    {
        h->slave[s].fault_summary_raw[0] = summary[s];
        h->slave[s].fault_summary_valid = true;
        if (summary[s] != 0U)
        {
            (void)BMS_RecordError(h,
                                  BMS_ERR_FAULT_ACTIVE,
                                  "SLAVE_FAULT",
                                  BMS_GetSlaveAddress(h, s),
                                  REG_SLAVE_FAULT_SUMMARY,
                                  0U,
                                  summary[s],
                                  0xFFU);
        }
    }
    for (uint8_t s = 0U; s < BMS_NUM_SLAVES; ++s)
    {
        uint32_t local = 0U;
        uint8_t sf = summary[s], dev = BMS_GetSlaveAddress(h, s);
        BMS_SlaveData_t *sl = &h->slave[s];
        if ((sf & (SLAVE_SUMMARY_PWR | SLAVE_SUMMARY_OTP | SLAVE_SUMMARY_COMP |
                   SLAVE_SUMMARY_PROT)) != 0U)
        {
            local |= BMS_FAULT_HARDWARE;
        }
        if ((sf & SLAVE_SUMMARY_COMM) != 0U)
        {
            local |= BMS_FAULT_COMM;
            st = BMS_ReadSingleDevice(h, dev, REG_SLAVE_FAULT_COMM1, &sl->fault_summary_raw[1], 3U);
            if (st != BMS_OK)
            {
                goto slave_failed;
            }
            sl->fault_comm_valid = true;
            if ((sl->fault_summary_raw[2] & SLAVE_COMM2_LINK_ERR) != 0U)
            {
                h->ring_intact = false;
                local |= BMS_FAULT_RING_BREAK;
            }
            if ((sl->fault_summary_raw[3] & SLAVE_COMM3_HB_ERRORS) != 0U)
            {
                local |= BMS_FAULT_HB_FAIL;
            }
        }
        if ((sf & (SLAVE_SUMMARY_OTP | SLAVE_SUMMARY_SYS)) != 0U)
        {
            st = BMS_ReadSingleDevice(
                h, dev, REG_SLAVE_FAULT_OTP, &sl->fault_detail_raw[BMS_FD_OTP], 2U);
            if (st != BMS_OK)
            {
                goto slave_failed;
            }
            sl->fault_detail_valid |= SLAVE_SUMMARY_OTP | SLAVE_SUMMARY_SYS;
            if ((sf & SLAVE_SUMMARY_SYS) != 0U)
            {
                uint8_t sys = sl->fault_detail_raw[BMS_FD_SYS];
                if ((sys & SLAVE_SYS_COMM_TIMEOUTS) != 0U)
                {
                    local |= BMS_FAULT_COMM;
                }
                if ((sys & (uint8_t)~SLAVE_SYS_COMM_TIMEOUTS) != 0U || sys == 0U)
                {
                    local |= BMS_FAULT_HARDWARE;
                }
            }
        }
        if ((sf & SLAVE_SUMMARY_PWR) != 0U)
        {
            st = BMS_ReadSingleDevice(
                h, dev, REG_SLAVE_FAULT_PWR1, &sl->fault_detail_raw[BMS_FD_PWR1], 3U);
            if (st != BMS_OK)
            {
                goto slave_failed;
            }
            sl->fault_detail_valid |= SLAVE_SUMMARY_PWR;
            /* Em runtime ate AVDD_OSC/AVDDUV_DRST sao falhas de hardware.
             * A excepcao e a limpeza verificada no fim do arranque. */
        }
        if ((sf & SLAVE_SUMMARY_PROT) != 0U)
        {
            st = BMS_ReadSingleDevice(
                h, dev, REG_SLAVE_FAULT_PROT1, &sl->fault_detail_raw[BMS_FD_PROT1], 2U);
            if (st != BMS_OK)
            {
                goto slave_failed;
            }
            sl->fault_detail_valid |= SLAVE_SUMMARY_PROT;
        }
        if ((sf & SLAVE_SUMMARY_OVUV) != 0U)
        {
            st = BMS_ReadSingleDevice(h, dev, REG_SLAVE_FAULT_OV1, buf, 4U);
            if (st != BMS_OK)
            {
                local |= BMS_FAULT_HARDWARE;
                goto slave_failed;
            }
            memcpy(sl->fault_ovuv_raw, buf, sizeof(sl->fault_ovuv_raw));
            sl->fault_ovuv_valid = true;
            /* Concatenar HI/LO forma uma mascara de 16 canais: bit zero corresponde a C1. */
            uint16_t ov = word_be(buf), uv = word_be(&buf[2]);
            if (ov == 0U && uv == 0U)
            {
                local |= BMS_FAULT_HARDWARE;
            }
            if (ov != 0U)
            {
                local |= BMS_FAULT_OV;
            }
            if (uv != 0U)
            {
                local |= BMS_FAULT_UV;
            }
            for (uint8_t c = 0U; c < BMS_CELLS_PER_SLAVE; ++c)
            {
                h->slave[s].ov_cell[c] = h->slave[s].ov_cell[c] || ((ov & (1U << c)) != 0U);
                h->slave[s].uv_cell[c] = h->slave[s].uv_cell[c] || ((uv & (1U << c)) != 0U);
            }
        }
        if ((sf & SLAVE_SUMMARY_OTUT) != 0U)
        {
            st = BMS_ReadSingleDevice(h, dev, REG_SLAVE_FAULT_OT, buf, 2U);
            if (st != BMS_OK)
            {
                local |= BMS_FAULT_HARDWARE;
                goto slave_failed;
            }
            memcpy(sl->fault_otut_raw, buf, sizeof(sl->fault_otut_raw));
            sl->fault_otut_valid = true;
            if (buf[0] != 0U)
            {
                local |= BMS_FAULT_OT;
            }
            if (buf[1] != 0U)
            {
                local |= BMS_FAULT_UT;
            }
            if ((buf[0] | buf[1]) == 0U)
            {
                local |= BMS_FAULT_HARDWARE;
            }
        }

        static const struct
        {
            uint16_t reg;
            uint8_t index, length;
        } comp[] = {{REG_SLAVE_FAULT_COMP_GPIO, BMS_FC_GPIO, 1U},
                    {REG_SLAVE_FAULT_COMP_VCCB1, BMS_FC_VCCB1, 2U},
                    {REG_SLAVE_FAULT_COMP_VCOW1, BMS_FC_VCOW1, 2U},
                    {REG_SLAVE_FAULT_COMP_CBOW1, BMS_FC_CBOW1, 2U},
                    {REG_SLAVE_FAULT_COMP_CBFET1, BMS_FC_CBFET1, 2U},
                    {REG_SLAVE_FAULT_COMP_MISC, BMS_FC_MISC, 1U}};

        for (size_t i = 0U; i < sizeof(comp) / sizeof(comp[0]); ++i)
        {
            /* Conservar a leitura VCOW que ja existia, mesmo sem COMP
             * no resumo. Os outros detalhes so sao pedidos com COMP. */
            if ((sf & SLAVE_SUMMARY_COMP) == 0U && comp[i].index != BMS_FC_VCOW1)
            {
                continue;
            }
            st = BMS_ReadSingleDevice(
                h, dev, comp[i].reg, &sl->fault_comp_raw[comp[i].index], comp[i].length);
            if (st != BMS_OK)
            {
                goto slave_failed;
            }
            sl->fault_comp_valid |= (uint16_t)(((1U << comp[i].length) - 1U) << comp[i].index);
        }
        if ((sf & SLAVE_SUMMARY_COMP) != 0U)
        {
            sl->fault_detail_valid |= SLAVE_SUMMARY_COMP;
        }
        if ((sl->fault_comp_raw[BMS_FC_VCOW1] | sl->fault_comp_raw[BMS_FC_VCOW2]) != 0U)
        {
            local |= BMS_FAULT_OPEN_WIRE;
            (void)BMS_RecordError(h,
                                  BMS_ERR_FAULT_ACTIVE,
                                  "VCOW",
                                  dev,
                                  REG_SLAVE_FAULT_COMP_VCOW1,
                                  0U,
                                  word_be(&sl->fault_comp_raw[BMS_FC_VCOW1]),
                                  0xFFFFU);
        }
        sl->fault_flags |= local;
        *faults |= local;
        continue;
    slave_failed:
        /* Conservar causas e bytes ja obtidos; scan incompleto nunca e valido. */
        sl->fault_flags |= local | fault_for_status(st);
        *faults |= local | fault_for_status(st);
        return st;
    }
    /* Somente o percurso completo renova o timestamp e aprova a validade do scan. */
    h->fault_scan_valid = true;
    h->last_fault_scan_ms = HAL_GetTick();
    return BMS_OK;
}

/**
 * @brief Trata NFAULT pendente ou ainda activo.
 *
 * Para que serve: nao regressar a saudavel apenas porque se consumiu a flag de interrupcao.
 *
 * Como funciona: inibe, recolhe causas e mantem FAULT. Um evento sem causa
 * explicada e tratado como hardware desconhecido, nao como sucesso.
 *
 * A recuperacao explicita/POST sao responsaveis por reconhecer uma causa resolvida.
 */
BMS_Status_t BMS_ProcessFaults(BMS_Handle_t *h)
{
    if (h == NULL)
    {
        return BMS_ERR_INVALID_PARAM;
    }
    if (h->state == BMS_STATE_SHUTDOWN || h->state == BMS_STATE_SLEEP)
    {
        BMS_InhibitOutputs(h);
        return BMS_ERR_NOT_READY;
    }
    if (!BMS_NfaultIsActive(h))
    {
        return h->fault_flags == 0U ? BMS_OK : BMS_ERR_FAULT_ACTIVE;
    }
    BMS_EmergencyShutdown(h); /* Primeiro inibir; so depois diagnosticar. */
    const uint32_t now = HAL_GetTick();
    if (h->nfault_diag_started &&
        (uint32_t)(now - h->last_nfault_diag_ms) < BMS_NFAULT_DIAG_PERIOD_MS)
    {
        return BMS_ERR_FAULT_ACTIVE;
    }
    h->last_nfault_diag_ms = now;
    h->nfault_diag_started = true;
    /* Consumir apenas quando se inicia um scan. IRQ durante a UART fica
     * pendente; todos os chamadores partilham a mesma cadencia. */
    /* Consumir o evento antes do scan: um novo flanco durante a UART fica pendente para a proxima passagem. */
    (void)__atomic_exchange_n(&h->nfault_pending, 0U, __ATOMIC_SEQ_CST);
    uint32_t faults = 0U;
    BMS_Status_t st = BMS_ScanHardwareFaults(h, &faults);
    if (st != BMS_OK)
    {
        faults |= fault_for_status(st);
    }
    if (faults == 0U)
    {
        faults = BMS_FAULT_HARDWARE;
        (void)BMS_RecordError(h,
                              BMS_ERR_FAULT_ACTIVE,
                              "NFAULT_UNEXPLAINED",
                              BMS_ADDR_BRIDGE,
                              0U,
                              GPIO_PIN_SET,
                              HAL_GPIO_ReadPin(BMS_NFAULT_PORT, BMS_NFAULT_PIN),
                              0U);
    }
    h->fault_flags |= faults;
    h->state = BMS_STATE_FAULT;
    return BMS_ERR_FAULT_ACTIVE;
}

/**
 * @brief Pede a limpeza dos latches de falha nos BQ.
 *
 * Para que serve: preparar uma nova verificacao depois de tratar a causa.
 *
 * Como funciona: usa FAULT_RST1/2 dos slaves e o FAULT_RST proprio da bridge;
 * invalida imediatamente o scan anterior, mesmo se a escrita falhar.
 *
 * Enviar a ordem nao prova que a causa desapareceu. Nao limpa fault_flags
 * do software nem reactiva permissoes; e necessario ler e validar de novo.
 */
BMS_Status_t BMS_ClearFaultLatches(BMS_Handle_t *h)
{
    if (h == NULL)
    {
        return BMS_ERR_INVALID_PARAM;
    }
    if (!h->addresses_valid || !h->direction_valid)
    {
        return BMS_ERR_NOT_READY;
    }
    BMS_InhibitOutputs(h);
    h->fault_scan_valid = false;
    uint8_t val = FAULT_RST1_ALL;
    BMS_Status_t st = BMS_WriteStack(h, REG_FAULT_RST1, &val, 1U);
    if (st != BMS_OK)
    {
        return st;
    }
    val = FAULT_RST2_ALL;
    st = BMS_WriteStack(h, REG_FAULT_RST2, &val, 1U);
    if (st != BMS_OK)
    {
        return st;
    }
    val = BRIDGE_FAULT_RST_ALL;
    st = BMS_WriteSingleDevice(h, BMS_ADDR_BRIDGE, REG_BRIDGE_FAULT_RST, &val, 1U);
    h->fault_scan_valid = false;
    return st; /* A causa so se considera ausente apos nova leitura/POST. */
}

/**
 * @brief Tenta recuperar um caminho DIR1 completo pelos dois slaves.
 *
 * Para que serve: restabelecer transporte sem confundir um unico slave com o pack todo.
 *
 * Como funciona: selecciona DIR1, confirma configuracao e le ambos. Conserva
 * FAULT ate ao reconhecimento posterior das causas e novo POST.
 *
 * Nao implementa dois segmentos independentes numa rotura entre S1 e S2.
 * Um retorno BMS_OK aqui significa transporte recuperado, nao rearme.
 */
BMS_Status_t BMS_RingRecovery(BMS_Handle_t *h)
{
    if (h == NULL)
    {
        return BMS_ERR_INVALID_PARAM;
    }
    BMS_EmergencyShutdown(h);
    h->post_passed = false;
    h->state = BMS_STATE_RING_RECOVERY;
    ++h->ring_recovery_count;
    h->ring_intact = false;
    h->voltage_valid = h->temperature_valid = h->inverter_voltage_valid = false;
    /* Politica deliberada: so recupera um caminho completo pelos dois slaves.
     * Se a rotura for central e exigir dois segmentos, permanece em FAULT.
     * NAO declarar sucesso com apenas um slave. Idempotente em DIR1. */
    BMS_Status_t st = BMS_SelectDirection(h, true);
    if (st == BMS_OK)
    {
        st = BMS_VerifyConfiguration(h);
    }
    if (st == BMS_OK)
    {
        st = BMS_ReadAllCellVoltages(h);
    }
    if (st != BMS_OK)
    {
        h->fault_flags |= BMS_FAULT_COMM | BMS_FAULT_RING_BREAK;
        h->direction_valid = false;
        BMS_EmergencyShutdown(h);
        return st;
    }
    h->fault_flags &= ~(uint32_t)BMS_FAULT_RING_BREAK;
    h->warning_flags |= BMS_WARN_RING_DEGRADED;
    /* Sucesso aqui significa TRANSPORTE recuperado. Requer limpar/confirmar
     * falhas e repetir POST antes de permitir operacao. */
    h->state = BMS_STATE_FAULT;
    return BMS_OK;
}

/**
 * @brief Retira a expectativa logica de contactor fechado.
 *
 * Coloca contactor_closed a false e pede paragem de balanceamento se este
 * estiver activo. Nao escreve GPIO nem confirma a abertura fisica do contactor.
 */
void BMS_ContactorOpen(BMS_Handle_t *h)
{
    if (h == NULL)
    {
        return;
    }
    h->contactor_closed = false;
    if (h->is_balancing)
    {
        h->balance_stop_pending = true;
    }
}

/**
 * @brief Recalcula as permissoes logicas bms_ok e precharge_ready.
 *
 * Exige modo operacional, POST/configuracao/diagnosticos aprovados, medicoes
 * recentes de tensao/temperatura/HV e ausencia de falhas. Confirma comunicacao
 * e estado de balanceamento de cada slave. A pre-carga compara bus e pack.
 * Publica os resultados numa secao critica curta para nao sobrepor uma
 * inibicao feita pela ISR. Em bancada bms_ok permanece false.
 * O nome e historico: esta funcao nao actua nos reles fisicos.
 */
void BMS_UpdateHardwareInterlocks(BMS_Handle_t *h)
{
    if (h == NULL)
    {
        return;
    }
    uint32_t now = HAL_GetTick();
    bool candidate =
        BMS_BENCH_MODE == 0U && h->post_passed && h->config_verified && h->diagnostics_confirmed &&
        h->addresses_valid && h->direction_valid && h->fault_flags == 0U && h->fault_scan_valid &&
        (uint32_t)(now - h->last_fault_scan_ms) <= BMS_HW_STATUS_MAX_AGE_MS &&
        BMS_MeasurementsFresh(h, true, true) &&
        (h->state == BMS_STATE_MONITORING || h->state == BMS_STATE_BALANCING) &&
        h->min_cell_mv > CELL_UV_MV && h->max_cell_mv < CELL_OV_MV &&
        h->max_temp_c != BMS_INVALID_TEMPERATURE_C && h->max_temp_c < (int16_t)CELL_TEMP_MAX_C;
    for (uint8_t s = 0U; s < BMS_NUM_SLAVES; ++s)
    {
        candidate = candidate && h->slave[s].comm_ok && h->slave[s].balance_state_known;
    }
    bool charged =
        candidate && h->pack_voltage_mv != 0U &&
        (uint64_t)h->inverter_voltage_mv * 100U >= (uint64_t)h->pack_voltage_mv * PRECHARGE_PERCENT;
    /* A ISR nao pode inibir entre o teste e a publicacao e ver a sua decisao
     * sobrescrita pelo contexto principal. Secao critica sem UART/esperas. */
    uint32_t irq = __get_PRIMASK();
    __disable_irq();
    candidate = candidate && !BMS_NfaultIsActive(h) &&
                __atomic_load_n(&h->safety_latched, __ATOMIC_SEQ_CST) == 0U;
    h->bms_ok = candidate;
    h->precharge_ready = candidate && charged;
    if (!candidate)
    {
        h->contactor_closed = false;
    }
    __set_PRIMASK(irq);
}

/**
 * @brief Actualiza a expectativa logica de fecho apos verificar as permissoes.
 *
 * Recalcula os interlocks, exige pre-carga e estado ENGAGED e volta a verificar
 * NFAULT/latch numa secao critica. contactor_closed reflecte a autorizacao,
 * nao uma medicao da posicao fisica. Nao envia qualquer comando GPIO.
 */
void BMS_ContactorClose(BMS_Handle_t *h)
{
    if (h == NULL)
    {
        return;
    }
    BMS_UpdateHardwareInterlocks(h);
    uint32_t irq = __get_PRIMASK();
    __disable_irq();
    bool permit = h->bms_ok && h->precharge_ready && !BMS_NfaultIsActive(h) &&
                  __atomic_load_n(&h->safety_latched, __ATOMIC_SEQ_CST) == 0U &&
                  BMS_Relays_GetState() == BMS_RLY_ENGAGED;
    h->contactor_closed = permit; /* Continua a ser expectativa, NAO feedback de contactor. */
    __set_PRIMASK(irq);
}

/**
 * @brief Resume o estado conhecido de balanceamento dos dois slaves.
 *
 * Para que serve: evitar apresentar uma paragem como confirmada quando houve erro UART.
 *
 * is_balancing mantem-se conservadoramente true se algum estado for desconhecido.
 */
static void update_balancing_summary(BMS_Handle_t *h)
{
    h->is_balancing = false;
    for (uint8_t s = 0U; s < BMS_NUM_SLAVES; ++s)
    {
        if (!h->slave[s].balance_state_known || h->slave[s].balance_running)
        {
            h->is_balancing = true;
        }
    }
}

/**
 * @brief Verifica as precondicoes existentes para iniciar balanceamento.
 *
 * Para que serve: centralizar as barreiras antes e depois das escritas de preparacao.
 *
 * O modo de bancada continua a bloquear o arranque. Nenhuma barreira
 * de temperatura/diagnostico e removida nesta revisao.
 */
static bool can_start_balancing(BMS_Handle_t *h)
{
    return BMS_BENCH_MODE == 0U && acquisition_ready(h) && h->post_passed &&
           h->diagnostics_confirmed && BMS_MeasurementsFresh(h, true, false) &&
           h->fault_flags == 0U && h->fault_scan_valid &&
           (uint32_t)(HAL_GetTick() - h->last_fault_scan_ms) <= BMS_HW_STATUS_MAX_AGE_MS &&
           (h->state == BMS_STATE_MONITORING || h->state == BMS_STATE_BALANCING) &&
           h->min_cell_mv > CELL_UV_MV && h->max_cell_mv < CELL_OV_MV &&
           h->max_temp_c != BMS_INVALID_TEMPERATURE_C &&
           h->max_temp_c < (int16_t)CELL_TEMP_WARN_C && !BMS_NfaultIsActive(h) &&
           __atomic_load_n(&h->safety_latched, __ATOMIC_SEQ_CST) == 0U;
}

/**
 * @brief Programa temporizadores por celula e confirma o estado do BQ.
 *
 * Para que serve: converter a mascara logica de C1..C15 nos registos CB_CELL16..1.
 *
 * Como funciona: preenche 16 timers, escreve em dois blocos de oito, confirma
 * por leitura, envia BAL_GO e verifica BAL_STAT. Mascara zero e a
 * sequencia de paragem: todos os timers zero e depois GO.
 *
 * BMS_OK inclui a confirmacao digital de estado, nao uma medicao da
 * corrente de balanceamento. Em erro o estado fica desconhecido.
 */
BMS_Status_t BMS_SetCellBalancing(BMS_Handle_t *h, uint8_t dev, uint16_t mask)
{
    if (h == NULL || (mask & 0x8000U) != 0U)
    {
        return BMS_ERR_INVALID_PARAM;
    }
    if (!h->addresses_valid || !h->direction_valid)
    {
        return BMS_ERR_NOT_READY;
    }
    uint8_t s;
    for (s = 0U; s < BMS_NUM_SLAVES; ++s)
    {
        if (dev == BMS_GetSlaveAddress(h, s))
        {
            break;
        }
    }
    if (s == BMS_NUM_SLAVES)
    {
        return BMS_ERR_INVALID_PARAM;
    }
    if (mask != 0U && !can_start_balancing(h))
    {
        return BMS_ERR_NOT_READY;
    }
    uint8_t timers[16U], rb[16U];
    /* A mascara numera C1 a partir do bit zero; os registos dos timers comecam por C16. */
    for (uint8_t c = 0U; c < 16U; ++c)
    {
        timers[15U - c] = (mask & (1U << c)) != 0U ? CB_TIMER_ACTIVE : CB_TIMER_OFF;
    }
    BMS_Status_t st;
    /* Garantir que o balanceamento de modulo nao fica activo em GPIO3. */
    const uint8_t zero = 0U;
    st = BMS_WriteSingleDevice(h, dev, REG_MB_TIMER_CTRL, &zero, 1U);
    if (st != BMS_OK)
    {
        goto failed;
    }
    st = BMS_WriteSingleDevice(h, dev, REG_CB_CELL16_CTRL, timers, 8U);
    if (st != BMS_OK)
    {
        goto failed;
    }
    st = BMS_WriteSingleDevice(h, dev, REG_CB_CELL16_CTRL + 8U, &timers[8], 8U);
    if (st != BMS_OK)
    {
        goto failed;
    }
    st = BMS_ReadSingleDevice(h, dev, REG_CB_CELL16_CTRL, rb, 16U);
    if (st != BMS_OK)
    {
        goto failed;
    }
    for (uint8_t i = 0U; i < sizeof(rb); ++i)
    {
        if (timers[i] != rb[i])
        {
            st = BMS_RecordError(h,
                                 BMS_ERR_INIT_FAILED,
                                 "BAL_TIMER_READBACK",
                                 dev,
                                 (uint16_t)(REG_CB_CELL16_CTRL + i),
                                 timers[i],
                                 rb[i],
                                 0xFFU);
            goto failed;
        }
    }
    if (mask != 0U && !can_start_balancing(h))
    {
        st = BMS_ERR_FAULT_ACTIVE;
        goto failed;
    }
    uint8_t ctrl = BAL_CTRL2_AUTO_GO;
    st = BMS_WriteSingleDevice(h, dev, REG_BAL_CTRL2, &ctrl, 1U);
    if (st != BMS_OK)
    {
        goto failed;
    }
    uint32_t start = HAL_GetTick();
    uint8_t status = 0U;
    for (uint32_t attempt = 0U; attempt <= BMS_BALANCE_VERIFY_TIMEOUT_MS; ++attempt)
    {
        st = BMS_ReadSingleDevice(h, dev, REG_BAL_STAT, &status, 1U);
        if (st != BMS_OK)
        {
            goto failed;
        }
        if (mask != 0U &&
            ((status & (BAL_STAT_ABORTFLT | BAL_STAT_INVALID_CONF | BAL_STAT_CB_PAUSED)) != 0U ||
             !can_start_balancing(h)))
        {
            st = BMS_ERR_FAULT_ACTIVE;
            goto failed;
        }
        bool run = (status & BAL_STAT_CB_RUN) != 0U;
        bool module_run = (status & BAL_STAT_MB_RUN) != 0U;
        if (!module_run && run == (mask != 0U))
        {
            h->slave[s].balance_mask = mask;
            h->slave[s].balance_running = run;
            h->slave[s].balance_state_known = true;
            if (run)
            {
                h->slave[s].last_balance_start_ms = HAL_GetTick();
                ++h->balance_cycle_count;
            }
            else
            {
                h->slave[s].last_balance_stop_ms = HAL_GetTick();
            }
            update_balancing_summary(h);
            return BMS_OK;
        }
        if ((uint32_t)(HAL_GetTick() - start) >= BMS_BALANCE_VERIFY_TIMEOUT_MS)
        {
            break;
        }
        BMS_DelayMs(1U);
    }
    st = BMS_RecordError(h,
                         BMS_ERR_TIMEOUT,
                         "BAL_STOP_START_TIMEOUT",
                         dev,
                         REG_BAL_STAT,
                         mask != 0U ? BAL_STAT_CB_RUN : 0U,
                         status,
                         BAL_STAT_CB_RUN | BAL_STAT_MB_RUN);
failed:
    /* TX concluido nao e ACK do BQ. Nao mentir sobre paragem/arranque. */
    h->slave[s].balance_state_known = false;
    h->fault_flags |= BMS_FAULT_BALANCING;
    if (st != BMS_ERR_FAULT_ACTIVE)
    {
        h->fault_flags |= fault_for_status(st);
    }
    update_balancing_summary(h);
    BMS_EmergencyShutdown(h);
    h->balance_stop_pending = true;
    return st;
}

/**
 * @brief Pede e confirma a paragem nos dois slaves.
 *
 * Para que serve: permitir parar mesmo quando novas sessoes estao bloqueadas.
 *
 * Como funciona: envia mascara zero a cada slave e agrega os resultados.
 *
 * Nao declara is_balancing=false apenas por ter tentado enviar a ordem.
 */
BMS_Status_t BMS_StopAllBalancing(BMS_Handle_t *h)
{
    if (h == NULL)
    {
        return BMS_ERR_INVALID_PARAM;
    }
    BMS_Status_t result = BMS_OK;
    for (uint8_t s = 0U; s < BMS_NUM_SLAVES; ++s)
    {
        BMS_Status_t st = BMS_SetCellBalancing(h, BMS_GetSlaveAddress(h, s), 0U);
        if (st != BMS_OK)
        {
            result = st;
        }
    }
    update_balancing_summary(h);
    h->balance_stop_pending = result != BMS_OK;
    return result;
}

/**
 * @brief Supervisiona as sessoes automaticas de balanceamento.
 *
 * Para que serve: seleccionar as celulas mais altas sem reiniciar o ciclo pares/impares
 * em cada passagem da tarefa.
 *
 * Como funciona: consulta a sessao em curso; decide paragem com valores
 * coerentes; so inicia uma nova sessao quando as condicoes existentes permitem.
 *
 * Nesta configuracao de bancada nao inicia sessoes. O codigo e conservado
 * para a integracao futura, sem activar as funcoes comentadas.
 */
BMS_Status_t BMS_RunPassiveBalancing(BMS_Handle_t *h)
{
    if (h == NULL)
    {
        return BMS_ERR_INVALID_PARAM;
    }
    if (!can_start_balancing(h))
    {
        return h->is_balancing || h->balance_stop_pending ? BMS_StopAllBalancing(h) : BMS_OK;
    }
    const uint32_t now = HAL_GetTick();
    for (uint8_t s = 0U; s < BMS_NUM_SLAVES; ++s)
    {
        BMS_SlaveData_t *sl = &h->slave[s];
        if (!sl->balance_state_known)
        {
            h->fault_flags |= BMS_FAULT_BALANCING;
            BMS_EmergencyShutdown(h);
            return BMS_ERR_FAULT_ACTIVE;
        }
        if (sl->balance_running)
        {
            uint8_t stat;
            BMS_Status_t st =
                BMS_ReadSingleDevice(h, BMS_GetSlaveAddress(h, s), REG_BAL_STAT, &stat, 1U);
            if (st != BMS_OK)
            {
                sl->balance_state_known = false;
                h->fault_flags |= fault_for_status(st);
                BMS_EmergencyShutdown(h);
                return st;
            }
            if ((stat & (BAL_STAT_ABORTFLT | BAL_STAT_INVALID_CONF | BAL_STAT_CB_PAUSED)) != 0U)
            {
                h->fault_flags |= BMS_FAULT_BALANCING;
                BMS_EmergencyShutdown(h);
                BMS_Status_t stop_status = BMS_StopAllBalancing(h);
                return stop_status == BMS_OK ? BMS_ERR_FAULT_ACTIVE : stop_status;
            }
            if ((stat & BAL_STAT_CB_RUN) == 0U)
            {
                sl->balance_running = false;
                sl->balance_mask = 0U;
                sl->last_balance_stop_ms = now;
                continue; /* So rearmar numa sessao futura, nao a cada 100 ms. */
            }
            bool stop = h->delta_cell_mv <= CELL_BALANCE_STOP_MV;
            /* Enquanto activo, continuar a supervisionar na faixa 5..19 mV. */
            for (uint8_t c = 0U; c < BMS_CELLS_PER_SLAVE; ++c)
            {
                if ((sl->balance_mask & (1U << c)) != 0U &&
                    (sl->cell_voltage_mv[c] <= CELL_BALANCE_MIN_MV ||
                     sl->cell_voltage_mv[c] <= h->min_cell_mv + CELL_BALANCE_STOP_MV))
                {
                    stop = true;
                }
            }
            if (stop)
            {
                st = BMS_SetCellBalancing(h, BMS_GetSlaveAddress(h, s), 0U);
                if (st != BMS_OK)
                {
                    return st;
                }
            }
            continue; /* Nao acrescentar canais reiniciando o ciclo pares/impares. */
        }
        if (h->delta_cell_mv < CELL_BALANCE_DELTA_MV ||
            (uint32_t)(now - sl->last_balance_stop_ms) < BMS_BALANCE_RESTART_DELAY_MS)
        {
            continue;
        }
        uint16_t mask = 0U;
        for (uint8_t c = 0U; c < BMS_CELLS_PER_SLAVE; ++c)
        {
            uint16_t mv = sl->cell_voltage_mv[c];
            if (mv > CELL_BALANCE_MIN_MV && mv > h->min_cell_mv + CELL_BALANCE_HYSTERESIS_MV)
            {
                mask |= (uint16_t)(1U << c);
            }
        }
        if (mask != 0U)
        {
            BMS_Status_t st = BMS_SetCellBalancing(h, BMS_GetSlaveAddress(h, s), mask);
            if (st != BMS_OK)
            {
                return st;
            }
        }
    }
    update_balancing_summary(h);
    if (h->fault_flags != 0U || BMS_NfaultIsActive(h))
    {
        BMS_EmergencyShutdown(h);
        return BMS_ERR_FAULT_ACTIVE;
    }
    h->state = h->is_balancing ? BMS_STATE_BALANCING : BMS_STATE_MONITORING;
    return BMS_OK;
}

/**
 * @brief Actualiza uma copia IIR para visualizacao.
 *
 * Para que serve: suavizar o display sem atrasar as comparacoes de proteccao por software.
 *
 * Como funciona: conserva o estado fraccionario Q16.16; arredonda apenas
 * a copia publicada. Degraus pequenos convergem nas duas direccoes.
 * A primeira amostra inicializa o estado. Proteccoes usam as amostras originais.
 *
 * cell_voltage_mv e as metricas usadas pelas proteccoes nao sao substituidas.
 */
void BMS_ApplyVoltageFilter(BMS_Handle_t *h)
{
    if (h == NULL || !BMS_MeasurementsFresh(h, false, false))
    {
        return;
    }
    uint16_t minimum = UINT16_MAX, maximum = 0U;
    uint32_t sum = 0U;
    for (uint8_t s = 0U; s < BMS_NUM_SLAVES; ++s)
    {
        for (uint8_t c = 0U; c < BMS_CELLS_PER_SLAVE; ++c)
        {
            /* Q16.16 guarda 16 bits de fraccao para que pequenos degraus nao fiquem presos em divisao inteira. */
            uint32_t target = (uint32_t)h->slave[s].cell_voltage_mv[c] * BMS_VOLTAGE_FILTER_SCALE;
            if (!h->filter_primed)
            {
                h->filtered_q16[s][c] = target;
            }
            else
            {
                /* int64_t evita overflow mesmo em degraus 0 <-> UINT16_MAX.
                 * A divisao assinada trunca para zero nas duas direccoes. */
                int64_t previous = h->filtered_q16[s][c];
                h->filtered_q16[s][c] =
                    (uint32_t)(previous +
                               ((int64_t)target - previous) / (int64_t)BMS_VOLTAGE_FILTER_ALPHA);
            }
            uint16_t filtered = (uint16_t)((h->filtered_q16[s][c] + BMS_VOLTAGE_FILTER_SCALE / 2U) /
                                           BMS_VOLTAGE_FILTER_SCALE);
            h->filtered_mv[s][c] = filtered;
            sum += filtered;
            if (filtered < minimum)
            {
                minimum = filtered;
            }
            if (filtered > maximum)
            {
                maximum = filtered;
            }
        }
    }
    /* A copia de amostras originais nunca e substituida pelo filtro. */
    h->filtered_pack_mv = sum;
    h->filtered_min_mv = minimum;
    h->filtered_max_mv = maximum;
    h->filtered_delta_mv = (uint16_t)(maximum - minimum);
    h->filter_primed = true;
}

/**
 * @brief Executa um ciclo de monitorizacao dos slaves.
 *
 * Para que serve: ordenar comunicacao, leitura, proteccoes e diagnostico periodico.
 *
 * Como funciona: trata paragem pendente/NFAULT; verifica configuracao; adquire
 * tensoes; verifica limites; le falhas; actualiza a copia filtrada e o estado.
 *
 * As linhas de temperaturas/HV continuam comentadas. A revisao nao
 * acciona reles nem muda o bloqueio de arranque do balanceamento.
 */
BMS_Status_t BMS_Task_100ms(BMS_Handle_t *hbms)
{
    BMS_Handle_t *h = hbms;
    if (h == NULL)
    {
        return BMS_ERR_INVALID_PARAM;
    }
    if (h->state == BMS_STATE_SHUTDOWN || h->state == BMS_STATE_SLEEP ||
        h->state == BMS_STATE_UNINITIALIZED || h->state == BMS_STATE_INITIALIZING)
    {
        BMS_InhibitOutputs(h);
        return BMS_ERR_NOT_READY;
    }
    if (h->balance_stop_pending)
    {
        BMS_Status_t st = BMS_StopAllBalancing(h);
        if (st != BMS_OK)
        {
            return st;
        }
    }
    if (BMS_NfaultIsActive(h))
    {
        return BMS_ProcessFaults(h);
    }
    /* Em FAULT nao retomar aquisicao normal: a aplicacao controla a tentativa de recuperacao. */
    if (h->state == BMS_STATE_FAULT)
    {
        BMS_InhibitOutputs(h);
        return BMS_ERR_FAULT_ACTIVE;
    }
    if (!h->config_verified ||
        (uint32_t)(HAL_GetTick() - h->last_config_check_ms) >= BMS_CONFIG_RECHECK_MS)
    {
        BMS_Status_t st = BMS_VerifyConfiguration(h);
        if (st != BMS_OK)
        {
            return st;
        }
    }
    BMS_Status_t status = BMS_ReadAllCellVoltages(h);
    if (status != BMS_OK)
    {
        return status;
    }
    //status = BMS_ReadAllTemperatures(hbms);
    //(void)BMS_ReadInverterVoltage(hbms);
    /* As duas chamadas acima continuam desactivadas. Em bancada nao permitem
     * balanceamento/BMS_OK; em operacional a ausencia de dados e uma falha. */
    status = BMS_CheckProtections(h);
    if (status != BMS_OK)
    {
        return status;
    }
    uint32_t faults = 0U;
    status = BMS_ScanHardwareFaults(h, &faults);
    if (status != BMS_OK || faults != 0U)
    {
        h->fault_flags |= faults;
        if (status != BMS_OK)
        {
            h->fault_flags |= fault_for_status(status);
        }
        BMS_EmergencyShutdown(h);
        return status == BMS_OK ? BMS_ERR_FAULT_ACTIVE : status;
    }
    if (BMS_NfaultIsActive(h))
    {
        return BMS_ProcessFaults(h);
    }
    BMS_ApplyVoltageFilter(h);
    /* Heuristica OCV preservada, nao uma medicao de SoC nem prova de repouso.
     * Falta medicao de corrente/tempo de relaxacao: soc_valid permanece false. */
    h->soc_valid = false;
    if (!h->contactor_closed && !h->is_balancing && BMS_Relays_GetState() != BMS_RLY_CHARGING)
    {
        h->soc_percent = BMS_EstimateSoC((uint16_t)(h->pack_voltage_mv / BMS_TOTAL_CELLS));
    }
    //status=BMS_RunPassiveBalancing(h);
    status = BMS_OK;
    /* Balanceamento automatico desactivado nesta revisao. */ //
    if (status != BMS_OK)
    {
        BMS_EmergencyShutdown(h);
        return status;
    }
    BMS_UpdateHardwareInterlocks(h);
    return BMS_OK;
}

/**
 * @brief Indica aquisicao de tensao validada dos dois slaves.
 *
 * Para que serve: mostrar slaves_ok no debug sem o confundir com autorizacao
 * do veiculo ou habilitacao de balanceamento.
 *
 * Como funciona: exige enderecos, configuracao/CRC, POST, ambas as leituras
 * recentes, scan de falhas recente, estado activo e NFAULT inactivo.
 *
 * E uma consulta: nao altera bms_ok, diagnostics_confirmed ou chamadas comentadas.
 * Nao exige NTC/HV para os ensaios de comunicacao e tensao desta entrega.
 */
bool BMS_SlavesReady(const BMS_Handle_t *h)
{
    if (h == NULL || !h->timer_valid || h->transport_quarantined || !h->addresses_valid ||
        !h->direction_valid || !h->config_verified || !h->post_passed || h->fault_flags != 0U ||
        !h->fault_scan_valid ||
        (h->state != BMS_STATE_MONITORING && h->state != BMS_STATE_BALANCING) ||
        !BMS_MeasurementsFresh(h, false, false) || BMS_NfaultIsActive(h) ||
        __atomic_load_n(&h->safety_latched, __ATOMIC_SEQ_CST) != 0U ||
        (uint32_t)(HAL_GetTick() - h->last_fault_scan_ms) > BMS_HW_STATUS_MAX_AGE_MS)
    {
        return false;
    }
    for (uint8_t s = 0U; s < BMS_NUM_SLAVES; ++s)
    {
        if (!h->slave[s].comm_ok || !h->slave[s].config_crc_valid)
        {
            return false;
        }
    }
    return true;
}

/**
 * @brief Converte o resultado de uma operacao em texto.
 *
 * Para que serve: distinguir TIMEOUT, CRC, FRAME, PLATFORM e os restantes resultados.
 *
 * Um estado de operacao nao e a mesma coisa que o conjunto fault_flags.
 */
const char *BMS_GetStatusString(BMS_Status_t status)
{
    switch (status)
    {
        case BMS_OK:
            return "OK";
        case BMS_ERR_TIMEOUT:
            return "TIMEOUT";
        case BMS_ERR_CRC:
            return "CRC";
        case BMS_ERR_COMM:
            return "COMM";
        case BMS_ERR_INVALID_PARAM:
            return "PARAM";
        case BMS_ERR_FAULT_ACTIVE:
            return "FAULT_ACTIVE";
        case BMS_ERR_INIT_FAILED:
            return "INIT_FAILED";
        case BMS_ERR_NOT_READY:
            return "NOT_READY";
        case BMS_ERR_FRAME:
            return "FRAME";
        case BMS_ERR_PLATFORM:
            return "PLATFORM";
        default:
            return "UNKNOWN";
    }
}

/**
 * @brief Converte o estado do driver em texto de debug.
 *
 * Para que serve: evitar numeros pouco legiveis no terminal.
 *
 * Devolve uma string estatica; nao comunica com os BQ.
 */
const char *BMS_GetStateString(BMS_State_t state)
{
    switch (state)
    {
        case BMS_STATE_UNINITIALIZED:
            return "UNINITIALIZED";
        case BMS_STATE_INITIALIZING:
            return "INITIALIZING";
        case BMS_STATE_IDLE:
            return "IDLE";
        case BMS_STATE_MONITORING:
            return "MONITORING";
        case BMS_STATE_BALANCING:
            return "BALANCING";
        case BMS_STATE_FAULT:
            return "FAULT";
        case BMS_STATE_SHUTDOWN:
            return "SHUTDOWN";
        case BMS_STATE_SLEEP:
            return "SLEEP";
        case BMS_STATE_RING_RECOVERY:
            return "RING_RECOVERY";
        default:
            return "UNKNOWN";
    }
}

/**
 * @brief Devolve uma descricao principal para o conjunto de falhas.
 *
 * Para que serve: mostrar rapidamente uma causa no debug antigo.
 *
 * Se houver varias flags, mostra apenas a primeira nesta ordem; consultar
 * o valor hexadecimal completo e os registos raw para nao perder informacao.
 */
const char *BMS_GetFaultString(uint32_t f)
{
    if (f == 0U)
        return "NONE";
    if (f & BMS_FAULT_OV)
        return "OVER_VOLTAGE";
    if (f & BMS_FAULT_UV)
        return "UNDER_VOLTAGE";
    if (f & BMS_FAULT_OT)
        return "OVER_TEMPERATURE";
    if (f & BMS_FAULT_UT)
        return "UNDER_TEMPERATURE";
    if (f & BMS_FAULT_PLATFORM)
        return "PLATFORM";
    if (f & BMS_FAULT_SENSOR)
        return "INVALID_SENSOR";
    if (f & BMS_FAULT_STALE_DATA)
        return "STALE_DATA";
    if (f & BMS_FAULT_CONFIG)
        return "CONFIGURATION";
    if (f & BMS_FAULT_HARDWARE)
        return "HARDWARE_FAULT";
    if (f & BMS_FAULT_BALANCING)
        return "BALANCING_UNKNOWN";
    if (f & BMS_FAULT_RING_BREAK)
        return "RING_BREAK";
    if (f & BMS_FAULT_OPEN_WIRE)
        return "OPEN_WIRE";
    if (f & BMS_FAULT_COMM)
        return "COMM_ERROR";
    if (f & BMS_FAULT_HB_FAIL)
        return "HEARTBEAT_FAIL";
    if (f & BMS_FAULT_CRC)
        return "CRC_ERROR";
    if (f & BMS_FAULT_FRAME)
        return "FRAME_ERROR";
    return "UNKNOWN_FAULT";
}

/* Curva OCV original mantida apenas como aproximacao de visualizacao. */
typedef struct
{
    uint16_t mv;
    uint8_t soc;
} SoC_Point_t;

static const SoC_Point_t g_soc_table[BMS_SOC_TABLE_SIZE] = {
    {3000U, 0U},  /* 3.00 V ->   0% (Subtensão Absoluta) */
    {3150U, 5U},  /* 3.15 V ->   5% */
    {3300U, 10U}, /* 3.30 V ->  10% */
    {3350U, 15U}, /* 3.35 V ->  15% */
    {3400U, 20U}, /* 3.40 V ->  20% */
    {3450U, 25U}, /* 3.45 V ->  25% */
    {3500U, 30U}, /* 3.50 V ->  30% */
    {3550U, 35U}, /* 3.55 V ->  35% */
    {3600U, 40U}, /* 3.60 V ->  40% */
    {3650U, 45U}, /* 3.65 V ->  45% */
    {3700U, 50U}, /* 3.70 V ->  50% (Tensão Nominal Termodinâmica) */
    {3750U, 55U}, /* 3.75 V ->  55% */
    {3800U, 60U}, /* 3.80 V ->  60% */
    {3850U, 65U}, /* 3.85 V ->  65% */
    {3900U, 70U}, /* 3.90 V ->  70% */
    {3950U, 75U}, /* 3.95 V ->  75% */
    {4000U, 80U}, /* 4.00 V ->  80% */
    {4050U, 85U}, /* 4.05 V ->  85% */
    {4100U, 90U}, /* 4.10 V ->  90% */
    {4150U, 95U}, /* 4.15 V ->  95% */
    {4200U, 100U} /* 4.20 V -> 100% (Totalmente Carregada) */
};

/**
 * @brief Estima uma percentagem de carga por interpolacao da curva OCV do projecto.
 *
 * Usa a tensao media de uma celula em milivolts, limita aos extremos da tabela
 * e interpola linearmente entre dois pontos. Nao mede corrente nem verifica
 * repouso, temperatura ou quimica; a tarefa mantem soc_valid=false.
 * Com celulas simuladas a 2 V, o resultado e zero pela saturacao inferior.
 *
 * @param avg_cell_mv Media das tensoes de celula, em mV; nao e a tensao total.
 * @return Estimativa entre 0 e 100, destinada apenas a visualizacao.
 */
uint8_t BMS_EstimateSoC(uint16_t avg_cell_mv)
{
    if (avg_cell_mv <= g_soc_table[0].mv)
    {
        return g_soc_table[0].soc;
    }
    if (avg_cell_mv >= g_soc_table[BMS_SOC_TABLE_SIZE - 1U].mv)
    {
        return g_soc_table[BMS_SOC_TABLE_SIZE - 1U].soc;
    }

    for (uint8_t i = 1U; i < BMS_SOC_TABLE_SIZE; i++)
    {
        if (avg_cell_mv <= g_soc_table[i].mv)
        {
            uint16_t mv_lo = g_soc_table[i - 1U].mv;
            uint16_t mv_hi = g_soc_table[i].mv;
            uint8_t soc_lo = g_soc_table[i - 1U].soc;
            uint8_t soc_hi = g_soc_table[i].soc;

            uint32_t delta_mv = (uint32_t)(avg_cell_mv - mv_lo);
            uint32_t range_mv = (uint32_t)(mv_hi - mv_lo);
            uint32_t delta_soc = (uint32_t)(soc_hi - soc_lo);

            if (range_mv == 0U)
            {
                return soc_lo;
            }
            return (uint8_t)(soc_lo + (uint8_t)((delta_mv * delta_soc) / range_mv));
        }
    }
    return 100U;
}
