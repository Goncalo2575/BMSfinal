/**
 * @file    bms_master_comm.c
 * @brief   BMS → Saída de DEBUG/Telemetria (USART2, PA2/PA3) — TX-only
 *
 */

#include "bms_master_comm.h"
#include "bms_relays.h"
#include <stdio.h>
#include <string.h>
#include <stdarg.h>

/**
 * @brief Associa a UART de debug e reinicia o contador de linhas.
 *
 * Para que serve: separar a USART2 de visualizacao da UART4 ligada a bridge.
 *
 * Como funciona: guarda apenas o handle; nao inicializa hardware e nao abre recepcao.
 *
 * Chamar depois da configuracao da USART2 no projecto STM32.
 */
void BMS_MasterComm_Init(BMS_MasterComm_t *hcomm, UART_HandleTypeDef *huart2)
{
    if (hcomm == NULL)
    {
        return;
    }
    hcomm->huart = NULL;
    hcomm->tx_count = 0U;
    if (huart2 == NULL)
    {
        return;
    }
    /* Nunca permitir que texto de debug seja enviado para a UART da BQ79600. */
    if (huart2->Instance != USART2)
    {
        return;
    }
    hcomm->huart = huart2;
}

/**
 * @brief Formata e envia uma linha de texto pela USART2 de debug.
 *
 * Usa um buffer local de 384 bytes e uma transmissao bloqueante limitada a
 * 50 ms. Recusa texto truncado e incrementa tx_count apenas em caso de sucesso.
 * Nao depende de printf/_write, nao recebe comandos e nao refresca o watchdog.
 * Chamar no contexto principal, com hcomm inicializado e formato valido.
 */
void BMS_MasterComm_Log(BMS_MasterComm_t *hcomm, const char *fmt, ...)
{
    if (hcomm == NULL || hcomm->huart == NULL || fmt == NULL)
    {
        return;
    }
    char line[384];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(line, sizeof(line), fmt, ap);
    va_end(ap);
    if (n <= 0 || (size_t)n >= sizeof(line))
    {
        return;
    }
    if (HAL_UART_Transmit(hcomm->huart, (uint8_t *)line, (uint16_t)n, 50U) == HAL_OK)
    {
        ++hcomm->tx_count;
    }
}

/**
 * @brief Envia a linha de telemetria geral usada pela integracao anterior.
 *
 * Mostra valores ja guardados no handle e a descricao principal das falhas.
 * A leitura dos monitores esta comentada: mon_valid=0 identifica os campos
 * de monitores/reles como nao adquiridos. Esta funcao nao faz leituras BQ.
 * A aplicacao actual usa slavestesteprint e mantem esta chamada comentada.
 */
void BMS_MasterComm_PrintDebug(BMS_MasterComm_t *hcomm, BMS_Handle_t *hbms)
{
    if ((hcomm == NULL) || (hbms == NULL) || (hcomm->huart == NULL))
    {
        return;
    }

    char line[384];

    /* Sem amostragem do modulo de reles, os campos seguintes sao placeholders (mon_valid=0). */
    BMS_RelayMonitors_t mon = {0};
    //BMS_Relays_GetMonitors(&mon);

    int n = snprintf(
        line,
        sizeof(line),
        "[BMS] st=%s flt=%s(0x%08lX) pack=%lumV min=%umV max=%umV "
        "dV=%umV Tmax=%dC SoC=%u%% HV=%lumV ring=%s ctor=%s ok=%u pre=%u "
        "cerr=%lu crc=%lu "
        "mon_valid=0 sfty=%s imd=%u tsms=%u esdb=%u esdbC=%u chgsig=%u "
        "rly[pre=%u dis=%u chg=%u bms=%u]\r\n",

        /* --- coisas que vêm dos BQs --- */
        BMS_GetStateString(hbms->state),       //state
        BMS_GetFaultString(hbms->fault_flags), //fault_flags
        (unsigned long)hbms->fault_flags,
        (unsigned long)hbms->pack_voltage_mv,        //pack_voltage_mv
        (unsigned)hbms->min_cell_mv,                 //min_cell_mv
        (unsigned)hbms->max_cell_mv,                 //max_cell_mv
        (unsigned)hbms->delta_cell_mv,               //delta_cell_mv
        (int)hbms->max_temp_c,                       //max_temp_c
        (unsigned)hbms->soc_percent,                 //soc_percent
        (unsigned long)hbms->inverter_voltage_mv,    //inverter_voltage_mv (B+)
        hbms->ring_intact ? "OK" : "BROKEN",         //ring_intact
        hbms->contactor_closed ? "CLOSE" : "OPEN",   /* Expectativa logica, sem feedback fisico. */
        (unsigned)(hbms->bms_ok ? 1U : 0U),          // interlock BMS_OK
        (unsigned)(hbms->precharge_ready ? 1U : 0U), // interlock PRECHARGE_OK
        (unsigned long)hbms->comm_error_count,       // comm_error_count
        (unsigned long)hbms->crc_error_count,        // crc_error_count

        /* --- malha de segurança / relés --- */
        BMS_Relays_GetStateString(),      // estado de segurança (SAFE/ENGAGED/CHARGING/NOT_SAFE)
        (unsigned)(mon.imd_ok ? 1U : 0U), // imd ok
        (unsigned)(mon.tsms ? 1U : 0U),   // tsms ok
        (unsigned)(mon.esdb ? 1U : 0U),   // esdb ok
        (unsigned)(mon.esdb_charger ? 1U : 0U),        // esdb charger ok
        (unsigned)(mon.charger ? 1U : 0U),             // charger ok
        (unsigned)(mon.pre_charge_closed ? 1U : 0U),   // pre-charge relay aberto/fechado
        (unsigned)(mon.discharge_closed ? 1U : 0U),    // discharge relay aberto/fechado
        (unsigned)(mon.charge_relay_closed ? 1U : 0U), // charge relay aberto/fechado
        (unsigned)(mon.bms_relay_closed ? 1U : 0U));   // BMS relay aberto/fechado

    if (n <= 0 || (size_t)n >= sizeof(line))
    {
        return;
    }
    uint16_t len = (uint16_t)n;

    if (HAL_UART_Transmit(hcomm->huart, (uint8_t *)line, len, 50U) == HAL_OK)
    {
        hcomm->tx_count++;
    }
}

/* Nao transmitir uma linha truncada: podia omitir precisamente o byte de falha. */
/**
 * @brief Envia uma linha completa de diagnostico, recusando truncamento.
 *
 * count e o retorno de snprintf; capacity e o tamanho total do buffer line.
 * Devolve false se o texto nao cabe ou se a transmissao falha, para que o
 * chamador pare o dump sem acumular varios timeouts de debug. Incrementa
 * tx_count apenas quando a HAL confirma o envio.
 */
static bool send_fault_line(BMS_MasterComm_t *hcomm, char *line, int count, size_t capacity)
{
    if (count <= 0 || (size_t)count >= capacity)
    {
        return false;
    }
    if (HAL_UART_Transmit(hcomm->huart, (uint8_t *)line, (uint16_t)count, 50U) != HAL_OK)
    {
        return false;
    }
    ++hcomm->tx_count;
    return true;
}

/**
 * @brief Mostra o ultimo snapshot de falhas da bridge e dos dois slaves.
 *
 * Imprime resumos, COMM, PWR/SYS/OTP/PROT, OV/UV/OT/UT e comparadores, sempre
 * com os respectivos indicadores de leitura valida. Inclui os bytes de wake
 * guardados antes da limpeza de arranque. Um byte zero sem validade nao prova
 * ausencia de falha. Nao faz novas transaccoes BQ nem refresca o IWDG.
 */
void BMS_MasterComm_PrintFaults(BMS_MasterComm_t *hcomm, const BMS_Handle_t *hbms)
{
    if (hcomm == NULL || hbms == NULL || hcomm->huart == NULL)
    {
        return;
    }
    char line[384];
    const uint8_t *bridge = hbms->bridge_fault_raw;
    int n = snprintf(
        line,
        sizeof(line),
        "[BQ-FAULT] scan_valid=%u bridge_read=%u SUM=%02X REG=%02X SYS=%02X PWR=%02X COMM=%02X/%02X\r\n",
        (unsigned)hbms->fault_scan_valid,
        (unsigned)hbms->bridge_fault_valid,
        (unsigned)bridge[0],
        (unsigned)bridge[1],
        (unsigned)bridge[2],
        (unsigned)bridge[3],
        (unsigned)bridge[4],
        (unsigned)bridge[5]);
    if (!send_fault_line(hcomm, line, n, sizeof(line)))
    {
        return;
    }
    for (uint8_t s = 0U; s < BMS_NUM_SLAVES; ++s)
    {
        const BMS_SlaveData_t *sl = &hbms->slave[s];
        const uint8_t *detail = sl->fault_detail_raw, *comp = sl->fault_comp_raw;
        n = snprintf(line,
                     sizeof(line),
                     "[S%u-FAULT] sumV=%u SUM=%02X commV=%u COMM=%02X/%02X/%02X detail=%02X "
                     "OTP=%02X SYS=%02X PWR=%02X/%02X/%02X PROT=%02X/%02X "
                     "wake(valid/cleared)=%u/%u:%02X/%02X/%02X\r\n",
                     (unsigned)(s + 1U),
                     (unsigned)sl->fault_summary_valid,
                     (unsigned)sl->fault_summary_raw[0],
                     (unsigned)sl->fault_comm_valid,
                     (unsigned)sl->fault_summary_raw[1],
                     (unsigned)sl->fault_summary_raw[2],
                     (unsigned)sl->fault_summary_raw[3],
                     (unsigned)sl->fault_detail_valid,
                     (unsigned)detail[BMS_FD_OTP],
                     (unsigned)detail[BMS_FD_SYS],
                     (unsigned)detail[BMS_FD_PWR1],
                     (unsigned)detail[BMS_FD_PWR2],
                     (unsigned)detail[BMS_FD_PWR3],
                     (unsigned)detail[BMS_FD_PROT1],
                     (unsigned)detail[BMS_FD_PROT2],
                     (unsigned)sl->init_pwr_valid,
                     (unsigned)sl->init_pwr_cleared,
                     (unsigned)sl->init_pwr_raw[0],
                     (unsigned)sl->init_pwr_raw[1],
                     (unsigned)sl->init_pwr_raw[2]);
        if (!send_fault_line(hcomm, line, n, sizeof(line)))
        {
            return;
        }
        n = snprintf(line,
                     sizeof(line),
                     "[S%u-OVUT] ovuvV=%u OV1/2=%02X/%02X UV1/2=%02X/%02X "
                     "otutV=%u OT/UT=%02X/%02X\r\n",
                     (unsigned)(s + 1U),
                     (unsigned)sl->fault_ovuv_valid,
                     (unsigned)sl->fault_ovuv_raw[0],
                     (unsigned)sl->fault_ovuv_raw[1],
                     (unsigned)sl->fault_ovuv_raw[2],
                     (unsigned)sl->fault_ovuv_raw[3],
                     (unsigned)sl->fault_otut_valid,
                     (unsigned)sl->fault_otut_raw[0],
                     (unsigned)sl->fault_otut_raw[1]);
        if (!send_fault_line(hcomm, line, n, sizeof(line)))
        {
            return;
        }
        n = snprintf(line,
                     sizeof(line),
                     "[S%u-COMP] read=%03X GPIO=%02X VCCB=%02X/%02X VCOW=%02X/%02X "
                     "CBOW=%02X/%02X CBFET=%02X/%02X MISC=%02X\r\n",
                     (unsigned)(s + 1U),
                     (unsigned)sl->fault_comp_valid,
                     (unsigned)comp[BMS_FC_GPIO],
                     (unsigned)comp[BMS_FC_VCCB1],
                     (unsigned)comp[BMS_FC_VCCB2],
                     (unsigned)comp[BMS_FC_VCOW1],
                     (unsigned)comp[BMS_FC_VCOW2],
                     (unsigned)comp[BMS_FC_CBOW1],
                     (unsigned)comp[BMS_FC_CBOW2],
                     (unsigned)comp[BMS_FC_CBFET1],
                     (unsigned)comp[BMS_FC_CBFET2],
                     (unsigned)comp[BMS_FC_MISC]);
        if (!send_fault_line(hcomm, line, n, sizeof(line)))
        {
            return;
        }
    }
}

/**
 * @brief Imprime um contexto FIRST ou LAST com a etiqueta recebida.
 *
 * Se o contexto nao existe, imprime valid=0. Caso contrario apresenta instante,
 * status, causa, etapa, direccao, dispositivo e exp/obs/mask. Os campos numericos
 * pertencem a operacao que falhou, nao a uma nova leitura efectuada pelo logger.
 */
static void
print_error_context(BMS_MasterComm_t *hcomm, const char *tag, const BMS_ErrorContext_t *ctx)
{
    if (ctx == NULL || !ctx->valid)
    {
        BMS_MasterComm_Log(hcomm, "[%s] valid=0\r\n", tag);
        return;
    }
    BMS_MasterComm_Log(hcomm,
                       "[%s] t=%lums st=%s reason=%s step=%s dir=%u dev=%02X reg=%04X "
                       "exp=%08lX obs=%08lX mask=%08lX\r\n",
                       tag,
                       (unsigned long)ctx->tick_ms,
                       BMS_GetStatusString(ctx->status),
                       ctx->reason ? ctx->reason : "?",
                       ctx->step ? ctx->step : "?",
                       (unsigned)ctx->direction,
                       (unsigned)ctx->device,
                       (unsigned)ctx->reg,
                       (unsigned long)ctx->expected,
                       (unsigned long)ctx->observed,
                       (unsigned long)ctx->mask);
}

/**
 * @brief Mostra um bloco de bytes em hexadecimal, em linhas de ate 16 bytes.
 *
 * tag distingue TX/RX e o sufixo indica o deslocamento no buffer. length e o
 * numero de bytes realmente disponiveis. Para no primeiro erro de transmissao
 * ou de formatacao, evitando consumir o orcamento do watchdog com novos timeouts.
 */
static void
print_hex(BMS_MasterComm_t *hcomm, const char *tag, const uint8_t *data, uint16_t length)
{
    if (hcomm == NULL || data == NULL)
    {
        return;
    }
    for (uint16_t base = 0U; base < length; base = (uint16_t)(base + 16U))
    {
        char line[160];
        int n = snprintf(line, sizeof(line), "[%s +%03u]", tag, (unsigned)base);
        if (n <= 0 || (size_t)n >= sizeof(line))
        {
            return;
        }
        size_t used = (size_t)n;
        uint16_t remaining = (uint16_t)(length - base);
        uint16_t count = remaining > 16U ? 16U : remaining;
        for (uint16_t i = 0U; i < count; ++i)
        {
            n = snprintf(&line[used], sizeof(line) - used, " %02X", (unsigned)data[base + i]);
            if (n <= 0 || (size_t)n >= sizeof(line) - used)
            {
                return;
            }
            used += (size_t)n;
        }
        if (sizeof(line) - used < 3U)
        {
            return;
        }
        line[used++] = '\r';
        line[used++] = '\n';
        line[used] = '\0';
        if (HAL_UART_Transmit(hcomm->huart, (uint8_t *)line, (uint16_t)used, 50U) != HAL_OK)
        {
            return;
        }
        ++hcomm->tx_count;
    }
}

/**
 * @brief Imprime FIRST/LAST, estado da plataforma e ultimo transporte falhado.
 *
 * Apresenta NFAULT fisico, flags DMA e idade do scan; quando existe captura de
 * erro, acrescenta quantidades esperadas/recebidas, codigos UART/DMA e bytes TX/RX.
 * Compara tx_count entre etapas para interromper no primeiro erro do terminal.
 * Consulta memoria e GPIO; nao pede novos registos aos BQ nem refresca IWDG.
 */
void BMS_MasterComm_PrintDiagnostics(BMS_MasterComm_t *hcomm, const BMS_Handle_t *hbms)
{
    if (hcomm == NULL || hbms == NULL || hcomm->huart == NULL)
    {
        return;
    }
    /* Parar no primeiro erro de debug: varias esperas HAL de 50 ms seguidas
     * nao podem consumir o orcamento do watchdog e apagar FIRST/LAST. */
    uint32_t sent = hcomm->tx_count;
    print_error_context(hcomm, "ERR-FIRST", &hbms->first_error);
    if (hcomm->tx_count == sent)
    {
        return;
    }
    sent = hcomm->tx_count;
    print_error_context(hcomm, "ERR-LAST", &hbms->last_error);
    if (hcomm->tx_count == sent)
    {
        return;
    }
    sent = hcomm->tx_count;
    BMS_MasterComm_Log(hcomm,
                       "[PLAT] timer=%u quarantined=%u busy=%lu dma_done=%lu dma_errflag=%lu "
                       "NFAULT(level/pending/epoch)=%u/%lu/%lu scan_age=%lums\r\n",
                       (unsigned)hbms->timer_valid,
                       (unsigned)hbms->transport_quarantined,
                       (unsigned long)hbms->comm_busy,
                       (unsigned long)hbms->dma_rx_done,
                       (unsigned long)hbms->dma_error,
                       (unsigned)HAL_GPIO_ReadPin(BMS_NFAULT_PORT, BMS_NFAULT_PIN),
                       (unsigned long)__atomic_load_n(&hbms->nfault_pending, __ATOMIC_SEQ_CST),
                       (unsigned long)__atomic_load_n(&hbms->nfault_epoch, __ATOMIC_SEQ_CST),
                       (unsigned long)(HAL_GetTick() - hbms->last_fault_scan_ms));
    if (hcomm->tx_count == sent)
    {
        return;
    }
    sent = hcomm->tx_count;
    if (hbms->debug_transport_valid)
    {
        BMS_MasterComm_Log(hcomm,
                           "[TR-ERR] t=%lums st=%s tx=%u rx=%u/%u uartErr=%08lX dmaErr=%08lX\r\n",
                           (unsigned long)hbms->debug_transport_tick_ms,
                           BMS_GetStatusString(hbms->debug_transport_status),
                           (unsigned)hbms->debug_tx_len,
                           (unsigned)hbms->debug_rx_received,
                           (unsigned)hbms->debug_rx_expected,
                           (unsigned long)hbms->debug_uart_error,
                           (unsigned long)hbms->debug_dma_error);
        if (hcomm->tx_count == sent)
        {
            return;
        }
        sent = hcomm->tx_count;
        print_hex(hcomm, "TX", hbms->debug_tx, hbms->debug_tx_len);
        if (hbms->debug_tx_len != 0U && hcomm->tx_count == sent)
        {
            return;
        }
        print_hex(hcomm, "RX", hbms->debug_rx, hbms->debug_rx_received);
    }
}

/**
 * @brief Mostra apenas bridge/slaves: estado, causa de erro e tensoes.
 *
 * Para que serve: no terminal, distinguir "os slaves respondem" de BMS_OK.
 * A linha global mostra direccao, etapa de arranque e validade. Seguem-se
 * duas linhas com a identidade fisica S1/S2 e C1..C15, em milivolts.
 * Nao faz novas leituras BQ. Valores antigos sao marcados com valid=0/age.
 * Cada transmissao e curta e limitada a 50 ms; tx_count conta so sucessos.
 */
void slavestesteprint(BMS_MasterComm_t *hcomm, BMS_Handle_t *hbms)
{
    if (hcomm == NULL || hbms == NULL || hcomm->huart == NULL)
    {
        return;
    }
    char line[384];
    //BMS_Relays_GetMonitors(&mon);
    const uint32_t now = HAL_GetTick();
    int n = snprintf(line,
                     sizeof(line),
                     "[BQ] fw=%u.%u.%u bench=%u st=%s slaves_ok=%u Vvalid=%u "
                     "addr=%u dirV=%u DIR%u ring=%u cfg=%u post=%u "
                     "pack=%lumV min=%u max=%u dV=%u flt=0x%08lX step=%s\r\n",
                     (unsigned)BMS_FW_VERSION_MAJOR,
                     (unsigned)BMS_FW_VERSION_MINOR,
                     (unsigned)BMS_FW_VERSION_PATCH,
                     (unsigned)BMS_BENCH_MODE,
                     BMS_GetStateString(hbms->state),
                     (unsigned)BMS_SlavesReady(hbms),
                     (unsigned)BMS_MeasurementsFresh(hbms, false, false),
                     (unsigned)hbms->addresses_valid,
                     (unsigned)hbms->direction_valid,
                     (unsigned)hbms->ring_using_reverse,
                     (unsigned)hbms->ring_intact,
                     (unsigned)hbms->config_verified,
                     (unsigned)hbms->post_passed,
                     (unsigned long)hbms->pack_voltage_mv,
                     (unsigned)hbms->min_cell_mv,
                     (unsigned)hbms->max_cell_mv,
                     (unsigned)hbms->delta_cell_mv,
                     (unsigned long)hbms->fault_flags,
                     hbms->init_step ? hbms->init_step : "UNKNOWN");
    if (n <= 0 || (size_t)n >= sizeof(line))
    {
        return;
    }
    uint16_t length = (uint16_t)n;
    if (HAL_UART_Transmit(hcomm->huart, (uint8_t *)line, length, 50U) != HAL_OK)
    {
        return;
    }
    ++hcomm->tx_count;
    for (uint8_t s = 0U; s < BMS_NUM_SLAVES; ++s)
    {
        const BMS_SlaveData_t *sl = &hbms->slave[s];
        /* Um numero guardado nao basta: Vvalid inclui tambem a idade da ultima aquisicao. */
        bool fresh = hbms->voltage_valid && sl->voltage_valid &&
                     (uint32_t)(now - sl->last_voltage_ms) <= BMS_MEASUREMENT_MAX_AGE_MS;
        n = snprintf(
            line,
            sizeof(line),
            "[S%u] addr0=%u addr1=%u now=%u comm=%u Vvalid=%u age=%lums crcV=%u crc=%02X%02X mV:",
            (unsigned)(s + 1U),
            (unsigned)sl->address,
            (unsigned)sl->address_rev,
            (unsigned)BMS_GetSlaveAddress(hbms, s),
            (unsigned)sl->comm_ok,
            (unsigned)fresh,
            (unsigned long)(now - sl->last_voltage_ms),
            (unsigned)sl->config_crc_valid,
            (unsigned)sl->expected_config_crc[0],
            (unsigned)sl->expected_config_crc[1]);
        if (n < 0 || (size_t)n >= sizeof(line))
        {
            return;
        }
        size_t used = (size_t)n;
        for (uint8_t cell = 0U; cell < BMS_CELLS_PER_SLAVE; ++cell)
        {
            n = snprintf(&line[used],
                         sizeof(line) - used,
                         " C%u=%u",
                         (unsigned)(cell + 1U),
                         (unsigned)sl->cell_voltage_mv[cell]);
            if (n < 0 || (size_t)n >= sizeof(line) - used)
            {
                return;
            }
            used += (size_t)n;
        }
        if (sizeof(line) - used < 3U)
        {
            return;
        }
        line[used++] = '\r';
        line[used++] = '\n';
        line[used] = '\0';
        if (HAL_UART_Transmit(hcomm->huart, (uint8_t *)line, (uint16_t)used, 50U) != HAL_OK)
        {
            return;
        }
        ++hcomm->tx_count;
    }
    uint32_t sent = hcomm->tx_count;
    BMS_MasterComm_PrintFaults(hcomm, hbms);
    if ((uint32_t)(hcomm->tx_count - sent) != 1U + 3U * BMS_NUM_SLAVES)
    {
        return;
    }
    BMS_MasterComm_PrintDiagnostics(hcomm, hbms);
}
