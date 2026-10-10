/**
 * @file bq796xx_bms.c
 * @brief Transporte UART, enderecamento e configuracao BQ79600/BQ79616.
 * Fontes: TI-600 pp.7,12,15-19,27-29,38-47; TI-616 pp.16,29,84,113-136.
 *
 * Percurso de uma leitura:
 * aplicacao -> BMS_ReadSingleDevice/ReadStack -> frame + CRC -> UART4
 * -> BQ79600 -> BQ79616 -> resposta DMA -> validacao -> dados do chamador.
 *
 * Organizacao: utilitarios, transporte, frames, enderecamento, configuracao
 * com CRC interno, inicializacao STM32 e testes basicos do arranque.
 *
 * IMPORTANTE: terminar uma escrita UART nao prova que o BQ a aceitou. As
 * configuracoes relevantes usam leitura de confirmacao; leituras so publicam
 * o resultado depois de validar tamanho, origem, registo e CRC.
 * Uma unica bridge, chamadas serializadas no contexto principal, nunca em ISR.
 */
#include "bq796xx_bms.h"

/* Destino dos callbacks HAL: apenas uma bridge e um handle neste driver.
 * Nao copiar o handle com uma recepcao DMA em curso. */
static BMS_Handle_t *g_hbms_irq = NULL;

/**
 * @brief Converte um resultado de operacao numa causa textual generica.
 *
 * Usada por BMS_RecordError quando o chamador nao fornece uma causa especifica.
 * Devolve um literal estatico; nao altera o handle nem comunica com os BQ.
 */
static const char *error_reason_for_status(BMS_Status_t status)
{
    switch (status)
    {
        case BMS_ERR_TIMEOUT:
            return "TIMEOUT";
        case BMS_ERR_CRC:
            return "CRC";
        case BMS_ERR_COMM:
            return "COMM";
        case BMS_ERR_INVALID_PARAM:
            return "INVALID_PARAM";
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
        case BMS_OK:
        default:
            return "OK";
    }
}

/**
 * @brief Converte um codigo de retorno numa categoria de falha do BMS.
 *
 * TIMEOUT e COMM tornam-se COMM; CRC, FRAME, CONFIG, PLATFORM e STALE_DATA
 * conservam categorias proprias. BMS_OK devolve zero. Os restantes erros
 * ficam classificados como HARDWARE. Nao le nem limpa registos do circuito.
 */
uint32_t BMS_FaultForStatus(BMS_Status_t status)
{
    if (status == BMS_OK)
    {
        return 0U;
    }
    if (status == BMS_ERR_INIT_FAILED)
    {
        return BMS_FAULT_CONFIG;
    }
    if (status == BMS_ERR_CRC)
    {
        return BMS_FAULT_CRC;
    }
    if (status == BMS_ERR_FRAME)
    {
        return BMS_FAULT_FRAME;
    }
    if (status == BMS_ERR_PLATFORM)
    {
        return BMS_FAULT_PLATFORM;
    }
    if (status == BMS_ERR_NOT_READY)
    {
        return BMS_FAULT_STALE_DATA;
    }
    if (status == BMS_ERR_COMM || status == BMS_ERR_TIMEOUT)
    {
        return BMS_FAULT_COMM;
    }
    return BMS_FAULT_HARDWARE;
}

/**
 * @brief Guarda a primeira causa e a causa mais recente de uma falha.
 *
 * FIRST so e preenchido uma vez; LAST e os campos antigos acompanham cada erro.
 * Guarda tambem etapa, instante e direccao, permitindo interpretar dev/reg no
 * terminal. Um resultado BMS_OK nao altera o historico.
 *
 * @param h Contexto persistente; se NULL, apenas devolve status.
 * @param status Resultado da operacao, devolvido sem alteracao.
 * @param reason Causa textual estatica; NULL usa a causa generica de status.
 * @param dev Endereco activo do chip; 0xFF representa Stack/Broadcast.
 * @param reg Endereco do registo envolvido, ou zero se nao aplicavel.
 * @param expected Valor esperado na verificacao que falhou.
 * @param observed Valor efectivamente observado.
 * @param mask Bits relevantes da comparacao; zero quando nao aplicavel.
 * @note reason e init_step sao ponteiros guardados, nao copias de texto.
 *       Devem apontar para texto que sobreviva ao retorno da funcao.
 */
BMS_Status_t BMS_RecordError(BMS_Handle_t *h,
                             BMS_Status_t status,
                             const char *reason,
                             uint8_t dev,
                             uint16_t reg,
                             uint32_t expected,
                             uint32_t observed,
                             uint32_t mask)
{
    if (h != NULL && status != BMS_OK)
    {
        BMS_ErrorContext_t ctx = {0};
        ctx.valid = true;
        ctx.status = status;
        ctx.reason = reason != NULL ? reason : error_reason_for_status(status);
        ctx.step = h->init_step;
        ctx.tick_ms = HAL_GetTick();
        ctx.direction = h->ring_using_reverse ? 1U : 0U;
        ctx.device = dev;
        ctx.reg = reg;
        ctx.expected = expected;
        ctx.observed = observed;
        ctx.mask = mask;
        /* LAST acompanha cada falha; FIRST conserva a causa inicial da sessao. */
        h->last_error = ctx;
        if (!h->first_error.valid)
        {
            h->first_error = ctx;
        }
        /* Campos antigos mantidos para compatibilidade com telemetria existente. */
        h->last_error_status = status;
        h->last_error_device = dev;
        h->last_error_reg = reg;
    }
    return status;
}

/**
 * @brief Limpa os contextos FIRST/LAST e a captura do ultimo transporte falhado.
 *
 * Inicializa os buffers TX/RX de diagnostico e os campos de compatibilidade.
 * Nao limpa fault_flags, nao envia resets aos BQ e nao rearma permissoes.
 * Chamar no contexto principal, sem actualizacao concorrente do historico.
 */
void BMS_ClearDebugHistory(BMS_Handle_t *h)
{
    if (h == NULL)
    {
        return;
    }
    memset(&h->first_error, 0, sizeof(h->first_error));
    memset(&h->last_error, 0, sizeof(h->last_error));
    h->last_error_status = BMS_OK;
    h->last_error_device = 0xFFU;
    h->last_error_reg = 0U;
    h->debug_transport_valid = false;
    h->debug_transport_status = BMS_OK;
    h->debug_transport_tick_ms = 0U;
    h->debug_uart_error = h->debug_dma_error = 0U;
    h->debug_tx_len = h->debug_rx_expected = h->debug_rx_received = 0U;
    memset(h->debug_tx, 0, sizeof(h->debug_tx));
    memset(h->debug_rx, 0, sizeof(h->debug_rx));
}

/**
 * @brief Conserva os bytes e erros da ultima transferencia que falhou.
 *
 * Copia TX e a parte realmente recebida de RX, limitando ambos aos buffers de
 * diagnostico. Guarda separadamente rx_expected e rx_received: um timeout pode
 * ter deixado apenas parte da resposta. Recebe os codigos UART/DMA capturados
 * antes de a HAL os limpar; BMS_OK e handle NULL nao alteram o snapshot.
 */
static void capture_transport_debug(BMS_Handle_t *h,
                                    BMS_Status_t status,
                                    const uint8_t *tx,
                                    uint16_t tx_len,
                                    const uint8_t *rx,
                                    uint16_t rx_expected,
                                    uint16_t rx_received,
                                    uint32_t uart_error,
                                    uint32_t dma_error)
{
    if (h == NULL || status == BMS_OK)
    {
        return;
    }
    h->debug_transport_valid = true;
    h->debug_transport_status = status;
    h->debug_transport_tick_ms = HAL_GetTick();
    h->debug_uart_error = uart_error;
    h->debug_dma_error = dma_error;
    /* Limitar as copias protege os buffers de diagnostico mesmo perante tamanhos invalidos. */
    h->debug_tx_len = tx_len <= BMS_MAX_FRAME_SIZE ? tx_len : BMS_MAX_FRAME_SIZE;
    h->debug_rx_expected = rx_expected;
    h->debug_rx_received = rx_received <= BMS_RX_BUFFER_SIZE ? rx_received : BMS_RX_BUFFER_SIZE;
    memset(h->debug_tx, 0, sizeof(h->debug_tx));
    memset(h->debug_rx, 0, sizeof(h->debug_rx));
    if (tx != NULL && h->debug_tx_len != 0U)
    {
        memcpy(h->debug_tx, tx, h->debug_tx_len);
    }
    if (rx != NULL && h->debug_rx_received != 0U)
    {
        memcpy(h->debug_rx, rx, h->debug_rx_received);
    }
}

/**
 * @brief Verifica os recursos minimos do transporte.
 *
 * Para que serve: recusar comunicacao sem UART/temporizador valido ou depois de um
 * abort DMA que deixou o transporte em estado incerto.
 *
 * Nao testa cabos nem a presenca dos BQ; isso exige respostas verificadas.
 */
static bool valid_handle(const BMS_Handle_t *h)
{
    return h != NULL && h->huart != NULL && h->htim_delay != NULL && h->timer_valid &&
           !h->transport_quarantined;
}

/**
 * @brief Espera em milissegundos pela base temporal HAL.
 *
 * Para que serve: usar as esperas de arranque em que nao e necessaria resolucao de
 * microssegundos.
 *
 * @param ms Duracao pedida. Nao chamar com a base HAL bloqueada ou numa ISR.
 * Nao refresca o watchdog dentro da espera.
 */
void BMS_DelayMs(uint32_t ms)
{
    HAL_Delay(ms);
}

/* TIM2 a 1 MHz, ARR=0xFFFFFFFF. Subtraccao unsigned suporta o wrap
 * (~71.6 minutos). O limite de iteracoes so detecta contador parado;
 * nunca e usado como substituto aproximado de um atraso temporal. */
/**
 * @brief Espera em microssegundos usando TIM2 em roda livre.
 *
 * Para que serve: gerar os tempos de WAKE, COMM CLEAR e mudanca de direccao sem
 * usar um ciclo de CPU como relogio.
 *
 * Como funciona: subtrai o contador inicial ao actual, suportando a passagem por
 * zero. Se o contador parar, regista PLATFORM e inibe as permissoes.
 *
 * @param h Handle inicializado com TIM2 a 1 MHz.
 * @param us Espera pedida, de 0 a 1000000 us; bloqueia o contexto principal.
 */
void BMS_DelayUs(BMS_Handle_t *h, uint32_t us)
{
    if (h == NULL)
    {
        return;
    }
    if (!h->timer_valid || h->htim_delay == NULL || us > 1000000U)
    {
        h->timer_valid = false;
        h->fault_flags |= BMS_FAULT_PLATFORM;
        (void)BMS_RecordError(h,
                              BMS_ERR_PLATFORM,
                              "TIM2_DELAY_INVALID",
                              0xFFU,
                              0U,
                              us,
                              h->htim_delay == NULL ? 0U : 1U,
                              0xFFFFFFFFUL);
        BMS_EmergencyShutdown(h);
        return;
    }
    const uint32_t start = __HAL_TIM_GET_COUNTER(h->htim_delay);
    uint32_t last = start, stagnant = 0U;
    for (;;)
    {
        uint32_t now = __HAL_TIM_GET_COUNTER(h->htim_delay);
        if ((uint32_t)(now - start) >= us)
        {
            return;
        }
        if (now == last)
        {
            if (++stagnant > 100000U)
            {
                h->timer_valid = false;
                h->fault_flags |= BMS_FAULT_PLATFORM;
                (void)BMS_RecordError(h,
                                      BMS_ERR_PLATFORM,
                                      "TIM2_STOPPED",
                                      0xFFU,
                                      0U,
                                      us,
                                      (uint32_t)(now - start),
                                      0xFFFFFFFFUL);
                BMS_EmergencyShutdown(h);
                return;
            }
        }
        else
        {
            last = now;
            stagnant = 0U;
        }
    }
}

/**
 * @brief Calcula o CRC utilizado nos frames BQ e na imagem de configuracao.
 *
 * Para que serve: detectar alteracoes nos bytes. A mesma rotina pode calcular o CRC
 * de um frame ou dos 54 bytes shadow; os dados de entrada sao diferentes.
 *
 * Como funciona: parte de 0xFFFF e aplica o polinomio reflectido 0xA001 a cada bit.
 *
 * @param data Bytes na ordem de transmissao/leitura.
 * @param length Numero de bytes, sem os dois bytes do proprio CRC.
 * @return CRC numerico; append_crc define a ordem dos bytes no frame.
 */
uint16_t BMS_CalculateCRC16(const uint8_t *data, uint16_t length)
{
    uint16_t crc = CRC16_INIT;
    if (data == NULL && length != 0U)
    {
        return 0U;
    }
    for (uint16_t i = 0U; i < length; ++i)
    {
        crc ^= data[i];
        for (uint8_t j = 0U; j < 8U; ++j)
        {
            crc = (crc & 1U) != 0U ? (uint16_t)((crc >> 1U) ^ CRC16_POLY_LSB_FIRST)
                                   : (uint16_t)(crc >> 1U);
        }
    }
    return crc;
}

/**
 * @brief Escolhe o endereco de uma identidade fisica no sentido actual.
 *
 * Para que serve: manter S1 e S2 associados as mesmas celulas ao inverter o anel.
 *
 * Como funciona: em DIR0 usa address; em DIR1 usa address_rev. Os campos nao
 * sao trocados entre si a cada recuperacao.
 *
 * @param index Indice fisico 0=S1, 1=S2.
 * @return Endereco activo, ou 0xFF para parametros invalidos.
 */
uint8_t BMS_GetSlaveAddress(const BMS_Handle_t *h, uint8_t index)
{
    if (h == NULL || index >= BMS_NUM_SLAVES)
    {
        return 0xFFU;
    }
    return h->ring_using_reverse ? h->slave[index].address_rev : h->slave[index].address;
}

/**
 * @brief Acrescenta dois bytes CRC ao fim de um frame.
 *
 * Para que serve: centralizar a ordem dos bytes e evitar variantes entre comandos.
 *
 * Como funciona: calcula sobre os bytes ja escritos, anexa o byte baixo e depois
 * o alto do resultado reflectido; actualiza length.
 *
 * Funcao interna: o buffer tem de reservar mais dois bytes.
 */
static void append_crc(uint8_t *frame, uint16_t *length)
{
    uint16_t crc = BMS_CalculateCRC16(frame, *length);
    frame[(*length)++] = (uint8_t)crc;
    frame[(*length)++] = (uint8_t)(crc >> 8U);
}

/* Nao desmonta a UART/DMA via DeInit/Init. Apenas muda TX para GPIO enquanto
 * nao existe transferencia. Pre-carrega HIGH antes de trocar a funcao do pino.
 * Mede a largura real de LOW; um WAKE demasiado alongado e rejeitado. */
/**
 * @brief Gera um ping LOW na linha que chega ao RX da bridge.
 *
 * Para que serve: acordar/desligar a bridge ou limpar o protocolo, sem transmitir
 * um frame UART normal.
 *
 * Como funciona: aborta RX, desliga temporariamente a UART, passa PA0 a GPIO,
 * gera o pulso e restaura AF8. Verifica o tempo pelo TIM2.
 *
 * @param low_us Tempo LOW pretendido. max_us=0 desactiva apenas o limite superior.
 * Nao constitui HW_RESET dos slaves. Apenas o contexto principal a pode usar.
 */
static BMS_Status_t
pulse_tx_mode(BMS_Handle_t *h, uint32_t low_us, uint32_t max_us, bool guard_low_with_irq_mask)
{
    uint32_t ipsr = __get_IPSR();
    uint32_t primask = __get_PRIMASK();
    if (ipsr != 0U)
    {
        return BMS_RecordError(
            h, BMS_ERR_PLATFORM, "PULSE_FROM_ISR", 0xFFU, 0U, 0U, ipsr, 0xFFFFFFFFUL);
    }
    if (primask != 0U)
    {
        return BMS_RecordError(
            h, BMS_ERR_PLATFORM, "PULSE_IRQ_DISABLED", 0xFFU, 0U, 0U, primask, 0xFFFFFFFFUL);
    }
    if (!valid_handle(h))
    {
        return BMS_ERR_INVALID_PARAM;
    }
    HAL_StatusTypeDef abort_status = HAL_UART_AbortReceive(h->huart);
    if (abort_status != HAL_OK)
    {
        h->transport_quarantined = true;
        return BMS_RecordError(h,
                               BMS_ERR_COMM,
                               "PULSE_DMA_ABORT",
                               0xFFU,
                               0U,
                               HAL_OK,
                               (uint32_t)abort_status,
                               0xFFFFFFFFUL);
    }
    __HAL_UART_DISABLE(h->huart);
    GPIO_InitTypeDef cfg = {0};
    cfg.Pin = BMS_BRIDGE_TX_PIN;
    cfg.Mode = GPIO_MODE_OUTPUT_PP;
    cfg.Pull = GPIO_NOPULL;
    cfg.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_WritePin(BMS_BRIDGE_TX_PORT, BMS_BRIDGE_TX_PIN, GPIO_PIN_SET);
    HAL_GPIO_Init(BMS_BRIDGE_TX_PORT, &cfg);
    BMS_DelayUs(h, 5U);
    uint32_t actual = 0U;
    if (!h->timer_valid)
    {
        goto restore_uart;
    }
    uint32_t irq_state = __get_PRIMASK();
    if (guard_low_with_irq_mask)
    {
        __disable_irq();
    }
    uint32_t start = __HAL_TIM_GET_COUNTER(h->htim_delay);
    HAL_GPIO_WritePin(BMS_BRIDGE_TX_PORT, BMS_BRIDGE_TX_PIN, GPIO_PIN_RESET);
    /* Um unico instante inicial: uma segunda funcao de atraso acrescentava
     * overhead a uma janela COMM CLEAR de apenas 15..20 us. */
    uint32_t last = start, stagnant = 0U;
    for (;;)
    {
        uint32_t now = __HAL_TIM_GET_COUNTER(h->htim_delay);
        if ((uint32_t)(now - start) >= low_us)
        {
            break;
        }
        if (now == last)
        {
            if (++stagnant > 100000U)
            {
                h->timer_valid = false;
                break;
            }
        }
        else
        {
            last = now;
            stagnant = 0U;
        }
    }
    HAL_GPIO_WritePin(BMS_BRIDGE_TX_PORT, BMS_BRIDGE_TX_PIN, GPIO_PIN_SET);
    actual = (uint32_t)(__HAL_TIM_GET_COUNTER(h->htim_delay) - start);
    if (guard_low_with_irq_mask)
    {
        __set_PRIMASK(irq_state);
    }
restore_uart:
    cfg.Mode = GPIO_MODE_AF_PP;
    cfg.Pull = GPIO_PULLUP;
    cfg.Alternate = BMS_BRIDGE_UART_AF;
    HAL_GPIO_Init(BMS_BRIDGE_TX_PORT, &cfg);
    __HAL_UART_CLEAR_OREFLAG(h->huart); /* Leitura SR/DR: descarta restos de RX. */
    __HAL_UART_ENABLE(h->huart);
    BMS_DelayUs(h, 5U);
    if (!h->timer_valid || (max_us != 0U && actual > max_us))
    {
        h->fault_flags |= BMS_FAULT_PLATFORM;
        (void)BMS_RecordError(h,
                              BMS_ERR_PLATFORM,
                              h->timer_valid ? "PULSE_TOO_LONG" : "PULSE_TIMER_INVALID",
                              0xFFU,
                              0U,
                              max_us,
                              actual,
                              0xFFFFFFFFUL);
        BMS_EmergencyShutdown(h);
        return BMS_ERR_PLATFORM;
    }
    return BMS_OK;
}

/**
 * @brief Gera um pulso de WAKE/SHUTDOWN mantendo interrupcoes activas.
 *
 * Delega em pulse_tx_mode. low_us e a duracao LOW pedida em microssegundos;
 * max_us e o limite superior aceite, ou zero para nao impor esse limite.
 * A rotina comum restaura a funcao UART do pino mesmo quando o pulso falha.
 */
static BMS_Status_t pulse_tx(BMS_Handle_t *h, uint32_t low_us, uint32_t max_us)
{
    return pulse_tx_mode(h, low_us, max_us, false);
}

/* COMM CLEAR dura apenas ~18 us; bloquear IRQ apenas durante o nivel LOW evita
 * que uma ISR prolongue o pulso para fora da janela. WAKE/SHUTDOWN nao usam
 * esta variante e continuam com interrupcoes activas. */
/**
 * @brief Gera COMM CLEAR com proteccao da curta janela LOW contra interrupcoes.
 *
 * Pede 18 us e rejeita duracao medida superior a 20 us. Apenas o intervalo LOW
 * fica protegido; WAKE e SHUTDOWN usam a variante pulse_tx.
 * Nao limpa os registos de falha: STOP/COMMCLR podem ficar registados na bridge.
 */
static BMS_Status_t pulse_comm_clear(BMS_Handle_t *h)
{
    return pulse_tx_mode(h, BMS_COMM_CLEAR_BITS, 20U, true);
}

/**
 * @brief Emite COMM CLEAR para recuperar a interface UART.
 *
 * Para que serve: permitir uma nova transaccao depois de timeout ou frame invalido.
 *
 * Como funciona: reserva o transporte e gera 18 periodos LOW a 1 Mbit/s. Nao
 * tenta sobrepor-se a outra transaccao.
 *
 * COMM CLEAR pode deixar indicadores STOP/COMMCLR; nao e uma limpeza geral
 * de falhas do BQ e nao deve ser usado para esconder uma causa persistente.
 */
void BMS_CommClear(BMS_Handle_t *h)
{
    if (!valid_handle(h))
    {
        return;
    }
    if (__atomic_exchange_n(&h->comm_busy, 1U, __ATOMIC_SEQ_CST) != 0U)
    {
        return;
    }
    /* O modo do driver e fixo a 1 Mbit/s: 18 periodos = 18 us. */
    BMS_Status_t st = pulse_comm_clear(h);
    if (st != BMS_OK)
    {
        h->fault_flags |= BMS_FAULT_COMM;
        BMS_InhibitOutputs(h);
    }
    __atomic_store_n(&h->comm_busy, 0U, __ATOMIC_SEQ_CST);
}

/* Half-duplex ao nivel de transaccao, embora TX/RX sejam pinos distintos.
 * O DMA usa exclusivamente h->rx_buf, nunca um buffer de stack que possa
 * deixar de existir depois de um erro de abort. */
/**
 * @brief Executa uma transaccao UART com resposta opcional por DMA.
 *
 * Para que serve: separar a transferencia fisica da interpretacao dos registos.
 *
 * Como funciona: 1) reserva o transporte; 2) prepara RX antes de TX; 3) transmite;
 * 4) espera resposta com timeout real; 5) aborta RX e liberta o transporte.
 * A validacao do conteudo do frame e feita pelo chamador.
 *
 * @return BMS_OK indica transferencia concluida, nao confirmacao de uma escrita
 * pelo BQ. Nao chamar em ISR, com interrupcoes desactivadas ou concorrentemente.
 */
static BMS_Status_t
BMS_Transceive(BMS_Handle_t *h, const uint8_t *tx, uint16_t tx_len, uint8_t *rx, uint16_t rx_len)
{
    /* Extrair o destino apenas para diagnostico; a validacao completa de parametros vem a seguir. */
    uint8_t request_dev = 0xFFU;
    uint16_t request_reg = 0U;
    if (tx != NULL && tx_len >= 6U && tx_len <= BMS_MAX_FRAME_SIZE)
    {
        uint8_t type = (uint8_t)(tx[0] & 0xF0U);
        bool single = type == INIT_BASE_SINGLE_READ || type == INIT_BASE_SINGLE_WRITE;
        uint8_t offset = single ? 2U : 1U;
        request_dev = single ? tx[1] : 0xFFU;
        request_reg = (uint16_t)(((uint16_t)tx[offset] << 8U) | tx[offset + 1U]);
    }
    uint32_t ipsr = __get_IPSR();
    uint32_t primask = __get_PRIMASK();
    if (ipsr != 0U)
    {
        return BMS_RecordError(h,
                               BMS_ERR_PLATFORM,
                               "TRANSCEIVE_FROM_ISR",
                               request_dev,
                               request_reg,
                               0U,
                               ipsr,
                               0xFFFFFFFFUL);
    }
    if (primask != 0U)
    {
        return BMS_RecordError(h,
                               BMS_ERR_PLATFORM,
                               "TRANSCEIVE_IRQ_DISABLED",
                               request_dev,
                               request_reg,
                               0U,
                               primask,
                               0xFFFFFFFFUL);
    }
    if (!valid_handle(h) || tx == NULL || tx_len < 6U || tx_len > BMS_MAX_FRAME_SIZE ||
        rx_len > BMS_RX_BUFFER_SIZE || ((rx == NULL) != (rx_len == 0U)))
    {
        return BMS_RecordError(
            h, BMS_ERR_INVALID_PARAM, "TRANSPORT_PARAM", request_dev, request_reg, 0U, tx_len, 0U);
    }
    if (__atomic_exchange_n(&h->comm_busy, 1U, __ATOMIC_SEQ_CST) != 0U)
    {
        return BMS_RecordError(
            h, BMS_ERR_COMM, "TRANSPORT_BUSY", request_dev, request_reg, 0U, 1U, 0U);
    }
    BMS_Status_t result = BMS_OK;
    bool rx_started = false;
    HAL_StatusTypeDef hal_status = HAL_UART_AbortReceive(h->huart);
    if (hal_status != HAL_OK)
    {
        h->transport_quarantined = true;
        result = BMS_ERR_COMM;
        (void)BMS_RecordError(h,
                              result,
                              "DMA_ABORT_BEFORE",
                              request_dev,
                              request_reg,
                              HAL_OK,
                              (uint32_t)hal_status,
                              0xFFFFFFFFUL);
        goto finish;
    }
    __HAL_UART_CLEAR_OREFLAG(h->huart);
    h->huart->ErrorCode = HAL_UART_ERROR_NONE;
    /* A verificacao e a limpeza dos latches formam uma unica operacao face
     * a ISR. Caso contrario uma IRQ entre ambas perderia a causa original. */
    __disable_irq();
    uint32_t pending_error = __atomic_exchange_n(&h->uart_error_pending, 0U, __ATOMIC_SEQ_CST);
    if (pending_error == 0U)
    {
        __atomic_store_n(&h->dma_rx_done, 0U, __ATOMIC_SEQ_CST);
        __atomic_store_n(&h->dma_error, 0U, __ATOMIC_SEQ_CST);
        __atomic_store_n(&h->dma_uart_error, 0U, __ATOMIC_SEQ_CST);
        __atomic_store_n(&h->dma_hw_error, 0U, __ATOMIC_SEQ_CST);
    }
    __set_PRIMASK(primask);
    if (pending_error != 0U)
    {
        h->fault_flags |= BMS_FAULT_COMM;
        BMS_EmergencyShutdown(h);
        result = BMS_ERR_COMM;
        (void)BMS_RecordError(h,
                              result,
                              "UART_ASYNC_ERROR",
                              request_dev,
                              request_reg,
                              0U,
                              __atomic_load_n(&h->dma_uart_error, __ATOMIC_SEQ_CST),
                              0U);
        goto finish;
    }

    /* Armar RX antes de transmitir evita perder o inicio de uma resposta rapida da bridge. */
    if (rx_len != 0U)
    {
        memset(h->rx_buf, 0, rx_len); /* Debug de timeout nao deve mostrar bytes antigos. */
        hal_status = HAL_UART_Receive_DMA(h->huart, h->rx_buf, rx_len);
        if (hal_status != HAL_OK)
        {
            result = BMS_ERR_COMM;
            (void)BMS_RecordError(h,
                                  result,
                                  "DMA_RX_START",
                                  request_dev,
                                  request_reg,
                                  HAL_OK,
                                  (uint32_t)hal_status,
                                  0xFFFFFFFFUL);
            goto finish;
        }
        rx_started = true;
    }
    hal_status = HAL_UART_Transmit(h->huart, (uint8_t *)tx, tx_len, BMS_UART_TIMEOUT_MS);
    if (hal_status != HAL_OK)
    {
        result = BMS_ERR_COMM;
        (void)BMS_RecordError(h,
                              result,
                              "UART_TX",
                              request_dev,
                              request_reg,
                              HAL_OK,
                              (uint32_t)hal_status,
                              0xFFFFFFFFUL);
        goto finish;
    }

    if (rx_started)
    {
        const uint32_t start = __HAL_TIM_GET_COUNTER(h->htim_delay);
        const uint32_t tick_start = HAL_GetTick();
        /* A 1 Mbit/s e 8N1, cada byte ocupa 10 us; somar a latencia prevista da cadeia. */
        const uint32_t timeout = (uint32_t)rx_len * 10U + BMS_DAISY_CHAIN_LATENCY_US;
        uint32_t last = start, stagnant = 0U;
        while (__atomic_load_n(&h->dma_rx_done, __ATOMIC_SEQ_CST) == 0U)
        {
            if (__atomic_load_n(&h->dma_error, __ATOMIC_SEQ_CST) != 0U ||
                h->huart->ErrorCode != HAL_UART_ERROR_NONE)
            {
                result = BMS_ERR_COMM;
                (void)BMS_RecordError(h,
                                      result,
                                      "UART_DMA_RX_ERROR",
                                      request_dev,
                                      request_reg,
                                      HAL_UART_ERROR_NONE,
                                      h->huart->ErrorCode |
                                          __atomic_load_n(&h->dma_uart_error, __ATOMIC_SEQ_CST),
                                      0xFFFFFFFFUL);
                goto finish;
            }
            uint32_t now = __HAL_TIM_GET_COUNTER(h->htim_delay);
            if ((uint32_t)(now - start) >= timeout ||
                (uint32_t)(HAL_GetTick() - tick_start) > timeout / 1000U + 2U)
            {
                result = BMS_ERR_TIMEOUT;
                (void)BMS_RecordError(h,
                                      result,
                                      "RX_TIMEOUT",
                                      request_dev,
                                      request_reg,
                                      timeout,
                                      (uint32_t)(now - start),
                                      0xFFFFFFFFUL);
                goto finish;
            }
            if (now == last)
            {
                if (++stagnant > 100000U)
                {
                    h->timer_valid = false;
                    result = BMS_ERR_PLATFORM;
                    (void)BMS_RecordError(h,
                                          result,
                                          "TIM2_STOPPED_RX",
                                          request_dev,
                                          request_reg,
                                          timeout,
                                          0U,
                                          0xFFFFFFFFUL);
                    goto finish;
                }
            }
            else
            {
                stagnant = 0U;
                last = now;
            }
        }
    }
    if (__atomic_load_n(&h->dma_error, __ATOMIC_SEQ_CST) != 0U ||
        h->huart->ErrorCode != HAL_UART_ERROR_NONE ||
        __HAL_UART_GET_FLAG(h->huart, UART_FLAG_ORE) != RESET)
    {
        result = BMS_ERR_COMM;
        (void)BMS_RecordError(h,
                              result,
                              "UART_RX_FLAGS",
                              request_dev,
                              request_reg,
                              HAL_UART_ERROR_NONE,
                              h->huart->ErrorCode |
                                  __atomic_load_n(&h->dma_uart_error, __ATOMIC_SEQ_CST),
                              0xFFFFFFFFUL);
    }

finish:;
    (void)__atomic_exchange_n(&h->uart_error_pending, 0U, __ATOMIC_SEQ_CST);
    /* Capturar o estado ANTES de abortar DMA/limpar a linha. Assim o terminal
     * mostra quantos bytes tinham realmente chegado e os ErrorCode originais. */
    uint16_t rx_received_snapshot = 0U;
    uint32_t uart_error_snapshot =
        h->huart->ErrorCode | __atomic_load_n(&h->dma_uart_error, __ATOMIC_SEQ_CST);
    uint32_t dma_error_snapshot = __atomic_load_n(&h->dma_hw_error, __ATOMIC_SEQ_CST);
    if (rx_started && h->huart->hdmarx != NULL)
    {
        uint32_t remaining = __HAL_DMA_GET_COUNTER(h->huart->hdmarx);
        if (remaining <= rx_len)
        {
            rx_received_snapshot = (uint16_t)(rx_len - remaining);
        }
        dma_error_snapshot |= h->huart->hdmarx->ErrorCode;
    }
    if (result == BMS_OK && rx_started && rx_received_snapshot != rx_len)
    {
        result = BMS_ERR_FRAME;
        (void)BMS_RecordError(h,
                              result,
                              "DMA_RX_INCOMPLETE",
                              request_dev,
                              request_reg,
                              rx_len,
                              rx_received_snapshot,
                              0U);
    }
    /* Tambem abortar apos falha do arranque DMA: nunca reutilizar DMA incerto. */
    hal_status = HAL_UART_AbortReceive(h->huart);
    if (hal_status != HAL_OK)
    {
        h->transport_quarantined = true;
        result = BMS_ERR_COMM;
        (void)BMS_RecordError(h,
                              result,
                              "DMA_ABORT_AFTER",
                              request_dev,
                              request_reg,
                              HAL_OK,
                              (uint32_t)hal_status,
                              0xFFFFFFFFUL);
    }
    /* Uma IRQ pode chegar entre a verificacao anterior e o abort. Capturar
     * de novo os latches da ISR antes de publicar o bloco recebido. */
    uart_error_snapshot |= __atomic_load_n(&h->dma_uart_error, __ATOMIC_SEQ_CST);
    dma_error_snapshot |= __atomic_load_n(&h->dma_hw_error, __ATOMIC_SEQ_CST);
    if (result == BMS_OK && (__atomic_load_n(&h->dma_error, __ATOMIC_SEQ_CST) != 0U ||
                             uart_error_snapshot != 0U || dma_error_snapshot != 0U))
    {
        result = BMS_ERR_COMM;
        (void)BMS_RecordError(h,
                              result,
                              "UART_DMA_LATE_ERROR",
                              request_dev,
                              request_reg,
                              0U,
                              uart_error_snapshot,
                              0U);
    }
    /* Publicar no buffer do chamador apenas depois de confirmar fim, tamanho e ausencia de erros. */
    if (result == BMS_OK && rx_len != 0U)
    {
        memcpy(rx, h->rx_buf, rx_len);
    }
    if (result != BMS_OK)
    {
        capture_transport_debug(h,
                                result,
                                tx,
                                tx_len,
                                h->rx_buf,
                                rx_len,
                                rx_received_snapshot,
                                uart_error_snapshot,
                                dma_error_snapshot);
        ++h->comm_error_count;
        if (result == BMS_ERR_PLATFORM)
        {
            h->fault_flags |= BMS_FAULT_PLATFORM;
        }
        BMS_InhibitOutputs(h);
        if (h->timer_valid && !h->transport_quarantined)
        {
            /* Dentro do lock: nao chamar BMS_CommClear, que adquire o mesmo lock. */
            if (pulse_comm_clear(h) != BMS_OK)
            {
                h->transport_quarantined = true;
            }
        }
    }
    else
    {
        BMS_DelayUs(h, BMS_WRITE_GAP_US);
        if (!h->timer_valid)
        {
            result = BMS_ERR_PLATFORM;
        }
    }
    __atomic_store_n(&h->comm_busy, 0U, __ATOMIC_SEQ_CST);
    return result;
}

/**
 * @brief Assinala o fim da recepcao DMA da bridge.
 *
 * Para que serve: libertar a espera de BMS_Transceive quando chegaram todos os bytes.
 *
 * Como funciona: so altera dma_rx_done se a UART do callback for a UART do BMS.
 *
 * Callback de interrupcao: nao executa novas leituras/escritas BQ.
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (g_hbms_irq != NULL && huart == g_hbms_irq->huart)
    {
        __atomic_store_n(&g_hbms_irq->dma_rx_done, 1U, __ATOMIC_SEQ_CST);
    }
}

/**
 * @brief Regista um erro de periferico UART durante a comunicacao.
 *
 * Para que serve: interromper a espera e impedir que um bloco com erro seja aceite.
 *
 * Como funciona: marca dma_error e inibe as permissoes; a tarefa principal trata
 * a limpeza e a comunicacao seguinte.
 *
 * Nao acciona reles nem envia frames dentro da interrupcao.
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (g_hbms_irq != NULL && huart == g_hbms_irq->huart)
    {
        __atomic_fetch_or(&g_hbms_irq->dma_uart_error, huart->ErrorCode, __ATOMIC_SEQ_CST);
        if (huart->hdmarx != NULL)
        {
            __atomic_fetch_or(
                &g_hbms_irq->dma_hw_error, huart->hdmarx->ErrorCode, __ATOMIC_SEQ_CST);
        }
        __atomic_store_n(&g_hbms_irq->dma_error, 1U, __ATOMIC_SEQ_CST);
        __atomic_store_n(&g_hbms_irq->uart_error_pending, 1U, __ATOMIC_SEQ_CST);
        BMS_InhibitOutputs(g_hbms_irq);
    }
}

/**
 * @brief Encaminha o flanco NFAULT para o handle da bridge.
 *
 * Para que serve: identificar um aviso de falha do hardware sem executar UART na ISR.
 *
 * Como funciona: filtra o pino PA8 e chama BMS_NFAULT_IRQHandler.
 *
 * Integrar outros utilizadores deste callback no projecto; nao duplicar a definicao.
 */
void HAL_GPIO_EXTI_Callback(uint16_t pin)
{
    if (pin == BMS_NFAULT_PIN && g_hbms_irq != NULL)
    {
        BMS_NFAULT_IRQHandler(g_hbms_irq);
    }
}

/**
 * @brief Constroi um comando de escrita Single, Stack ou Broadcast.
 *
 * Para que serve: reutilizar a validacao de parametros e a montagem do frame.
 *
 * Como funciona: INIT -> DEV_ADR apenas no Single -> REG_HI/LO -> 1..8 dados
 * -> CRC. Envia sem esperar resposta, porque escritas nao geram ACK.
 *
 * Os wrappers publicos seleccionam o tipo de comando; reg e endereco,
 * data e conteudo, dev e identificador do chip. Sao conceitos distintos.
 */
static BMS_Status_t write_frame(BMS_Handle_t *h,
                                uint8_t init,
                                bool single,
                                uint8_t dev,
                                uint16_t reg,
                                const uint8_t *data,
                                uint8_t n)
{
    if (!valid_handle(h) || data == NULL || n == 0U || n > BMS_MAX_WRITE_BYTES ||
        (single && dev > BMS_DEVICE_ADDR_MAX) || (uint32_t)reg + n > 65536UL)
    {
        return BMS_ERR_INVALID_PARAM;
    }
    uint8_t frame[BMS_MAX_FRAME_SIZE];
    uint16_t k = 0U;
    frame[k++] = BMS_INIT(init, n);
    if (single)
    {
        frame[k++] = dev;
    }
    frame[k++] = (uint8_t)(reg >> 8U);
    frame[k++] = (uint8_t)reg;
    memcpy(&frame[k], data, n);
    k += n;
    append_crc(frame, &k);
    return BMS_Transceive(h, frame, k, NULL, 0U);
}

/**
 * @brief Escreve de um a oito registos de um unico dispositivo.
 *
 * Para que serve: configurar a bridge ou um slave sem aplicar a ordem aos restantes.
 *
 * Como funciona: usa INIT Single Write e inclui DEV_ADR no frame.
 *
 * @param dev Endereco 0..63; na rede actual 0=bridge, 1/2=slaves.
 * @param reg Primeiro registo. n bytes ocupam reg..reg+n-1.
 * @return Estado do envio; usar readback quando for preciso confirmar a escrita.
 */
BMS_Status_t
BMS_WriteSingleDevice(BMS_Handle_t *h, uint8_t dev, uint16_t reg, const uint8_t *data, uint8_t n)
{
    return write_frame(h, INIT_BASE_SINGLE_WRITE, true, dev, reg, data, n);
}

/**
 * @brief Envia uma escrita Broadcast normal.
 *
 * Para que serve: distribuir configuracao/endereco por toda a cadeia durante o arranque.
 *
 * Como funciona: nao inclui DEV_ADR; INIT tem o tipo Broadcast Write e tamanho N-1.
 *
 * A bridge tambem processa registos que existem no seu mapa. Nao confundir
 * Broadcast com Stack Write, que se destina aos dispositivos da stack.
 */
BMS_Status_t BMS_WriteBroadcast(BMS_Handle_t *h, uint16_t reg, const uint8_t *data, uint8_t n)
{
    return write_frame(h, INIT_BASE_BROADCAST_WRITE, false, 0U, reg, data, n);
}

/**
 * @brief Envia a mesma escrita aos dispositivos da stack.
 *
 * Para que serve: escrever nos slaves sem tratar a bridge como um monitor celular.
 *
 * Como funciona: monta um frame Stack Write, sem DEV_ADR.
 *
 * Usado na sincronizacao DLL e na limpeza de falhas dos slaves.
 */
BMS_Status_t BMS_WriteStack(BMS_Handle_t *h, uint16_t reg, const uint8_t *data, uint8_t n)
{
    return write_frame(h, INIT_BASE_STACK_WRITE, false, 0U, reg, data, n);
}

/**
 * @brief Muda DIR_SEL dos slaves pela porta oposta.
 *
 * Para que serve: atravessar a fase em que bridge e slaves ainda nao concordam
 * sobre o sentido de comunicacao.
 *
 * Como funciona: aceita apenas CONTROL1 e os dados 0x00 ou 0x80. ADDR_WR
 * e os novos enderecos usam Broadcast normal, nao esta funcao.
 *
 * Nao e uma alternativa generica a Broadcast Write. Requer a sequencia
 * de mudanca de direccao e as esperas descritas em change_direction.
 */
BMS_Status_t BMS_WriteBroadcastReverse(
    BMS_Handle_t *h, uint16_t reg, uint8_t init, const uint8_t *data, uint8_t n)
{
    /* Reverse SO para mudar DIR_SEL; ADDR_WR usa Broadcast normal. */
    if (init != INIT_BASE_BCAST_REV_WRITE || reg != REG_BRIDGE_CONTROL1 || n != 1U ||
        data == NULL || (data[0] != 0U && data[0] != CTRL1_DIR_SEL))
    {
        return BMS_ERR_INVALID_PARAM;
    }
    return write_frame(h, init, false, 0U, reg, data, n);
}

/**
 * @brief Valida a integridade e o cabecalho de uma resposta.
 *
 * Para que serve: recusar uma resposta corrompida ou pertencente a outro registo/tamanho.
 *
 * Como funciona: verifica CRC, INIT de resposta, tamanho, endereco e registo.
 * dev=0xFF aceita qualquer endereco valido; nesse caso ReadStack confirma
 * depois a identidade fisica e recusa respostas repetidas.
 *
 * CRC correcto sozinho nao prova que respondeu o dispositivo pretendido.
 */
static BMS_Status_t
validate_response(BMS_Handle_t *h, const uint8_t *f, uint8_t dev, uint16_t reg, uint8_t n)
{
    uint16_t crc = BMS_CalculateCRC16(f, (uint16_t)(n + 4U));
    uint16_t received = (uint16_t)f[n + 4U] | ((uint16_t)f[n + 5U] << 8U);
    if (crc != received)
    {
        ++h->crc_error_count;
        return BMS_RecordError(h, BMS_ERR_CRC, "FRAME_CRC", dev, reg, crc, received, 0xFFFFU);
    }
    if (f[0] != (uint8_t)(n - 1U))
    {
        return BMS_RecordError(h, BMS_ERR_FRAME, "FRAME_LENGTH", dev, reg, n - 1U, f[0], 0xFFU);
    }
    if (f[1] > BMS_DEVICE_ADDR_MAX || (dev != 0xFFU && f[1] != dev))
    {
        return BMS_RecordError(h, BMS_ERR_FRAME, "FRAME_DEVICE", dev, reg, dev, f[1], 0xFFU);
    }
    uint16_t got_reg = (uint16_t)(((uint16_t)f[2] << 8U) | f[3]);
    if (got_reg != reg)
    {
        return BMS_RecordError(h, BMS_ERR_FRAME, "FRAME_REGISTER", dev, reg, reg, got_reg, 0xFFFFU);
    }
    return BMS_OK;
}

/**
 * @brief Le de 1 a 128 bytes de um dispositivo.
 *
 * Para que serve: obter registos da bridge ou de um slave identificado.
 *
 * Como funciona: monta Single Read, pede N-1, recebe N+6 bytes, valida cabecalho,
 * origem e CRC, e so entao copia N bytes para data.
 *
 * @param data Buffer do chamador com pelo menos n bytes.
 * Numa falha, nao publica uma resposta parcial como valida.
 */
BMS_Status_t
BMS_ReadSingleDevice(BMS_Handle_t *h, uint8_t dev, uint16_t reg, uint8_t *data, uint8_t n)
{
    if (!valid_handle(h) || data == NULL || n == 0U || n > BMS_MAX_READ_BYTES ||
        dev > BMS_DEVICE_ADDR_MAX || (uint32_t)reg + n > 65536UL)
    {
        return BMS_ERR_INVALID_PARAM;
    }
    uint8_t tx[7U], rx[BMS_MAX_READ_BYTES + BMS_RESPONSE_OVERHEAD];
    uint16_t k = 0U;
    tx[k++] = INIT_BASE_SINGLE_READ;
    tx[k++] = dev;
    tx[k++] = (uint8_t)(reg >> 8U);
    tx[k++] = (uint8_t)reg;
    tx[k++] = (uint8_t)(n - 1U);
    append_crc(tx, &k);
    BMS_Status_t st = BMS_Transceive(h, tx, k, rx, (uint16_t)(n + BMS_RESPONSE_OVERHEAD));
    if (st != BMS_OK)
    {
        return st;
    }
    st = validate_response(h, rx, dev, reg, n);
    if (st != BMS_OK)
    {
        capture_transport_debug(h,
                                st,
                                tx,
                                k,
                                rx,
                                (uint16_t)(n + BMS_RESPONSE_OVERHEAD),
                                (uint16_t)(n + BMS_RESPONSE_OVERHEAD),
                                0U,
                                0U);
        BMS_InhibitOutputs(h);
        BMS_CommClear(h);
        return st;
    }
    memcpy(data, &rx[4], n); /* So publicar depois de validar TODO o frame. */
    return BMS_OK;
}

/**
 * @brief Le o mesmo bloco dos dois slaves.
 *
 * Para que serve: adquirir dados da stack sem uma resposta adicional da bridge.
 *
 * Como funciona: recebe dois frames completos, valida cada um e identifica S1/S2
 * pelo endereco. Recusa duplicados e publica so depois de ambos serem validos.
 *
 * @param data Buffer com pelo menos BMS_NUM_SLAVES*n bytes.
 * O bloco de S1 fica primeiro, mesmo quando DIR1 altera a ordem no cabo.
 */
BMS_Status_t BMS_ReadStack(BMS_Handle_t *h, uint16_t reg, uint8_t *data, uint8_t n)
{
    if (!valid_handle(h) || data == NULL || n == 0U || n > BMS_MAX_READ_BYTES ||
        (uint32_t)reg + n > 65536UL)
    {
        return BMS_ERR_INVALID_PARAM;
    }
    uint8_t tx[6U], rx[BMS_RX_BUFFER_SIZE], mapping[BMS_NUM_SLAVES];
    uint16_t k = 0U, size = (uint16_t)(n + BMS_RESPONSE_OVERHEAD);
    /* seen e um bitmap de identidades fisicas; mapping associa cada frame recebido a S1/S2. */
    uint8_t seen = 0U;
    for (uint8_t s = 0U; s < BMS_NUM_SLAVES; ++s)
    {
        h->slave[s].comm_ok = false;
    }
    tx[k++] = INIT_BASE_STACK_READ;
    tx[k++] = (uint8_t)(reg >> 8U);
    tx[k++] = (uint8_t)reg;
    tx[k++] = (uint8_t)(n - 1U);
    append_crc(tx, &k);
    BMS_Status_t st = BMS_Transceive(h, tx, k, rx, (uint16_t)(size * BMS_NUM_SLAVES));
    if (st != BMS_OK)
    {
        return st;
    }
    /* Identidade, comprimento, registo, CRC e ausencia de duplicados.
     * A atribuicao logica NAO depende apenas da posicao no fluxo. */
    for (uint8_t w = 0U; w < BMS_NUM_SLAVES; ++w)
    {
        const uint8_t *f = &rx[w * size];
        st = validate_response(h, f, 0xFFU, reg, n);
        if (st != BMS_OK)
        {
            goto invalid;
        }
        uint8_t s;
        for (s = 0U; s < BMS_NUM_SLAVES; ++s)
        {
            if (f[1] == BMS_GetSlaveAddress(h, s))
            {
                break;
            }
        }
        if (s == BMS_NUM_SLAVES || (seen & (uint8_t)(1U << s)) != 0U)
        {
            st = BMS_RecordError(
                h, BMS_ERR_FRAME, "STACK_DEVICE_DUP_OR_UNKNOWN", 0xFFU, reg, seen, f[1], 0U);
            goto invalid;
        }
        seen |= (uint8_t)(1U << s);
        mapping[w] = s;
    }
    if (seen != (uint8_t)((1U << BMS_NUM_SLAVES) - 1U))
    {
        st = BMS_RecordError(h,
                             BMS_ERR_FRAME,
                             "STACK_MISSING_DEVICE",
                             0xFFU,
                             reg,
                             (1U << BMS_NUM_SLAVES) - 1U,
                             seen,
                             0U);
        goto invalid;
    }
    for (uint8_t w = 0U; w < BMS_NUM_SLAVES; ++w)
    {
        memcpy(&data[mapping[w] * n], &rx[w * size + 4U], n);
        h->slave[mapping[w]].comm_ok = true;
    }
    return BMS_OK;
invalid:
    capture_transport_debug(h,
                            st,
                            tx,
                            k,
                            rx,
                            (uint16_t)(size * BMS_NUM_SLAVES),
                            (uint16_t)(size * BMS_NUM_SLAVES),
                            0U,
                            0U);
    BMS_InhibitOutputs(h);
    BMS_CommClear(h);
    return st;
}

/**
 * @brief Envia as oito escritas de sincronizacao DLL indicadas pela TI.
 *
 * Para que serve: preparar os relogios da interface daisy-chain antes de enderecar.
 *
 * Como funciona: escreve 0x00 separadamente em 0x0343..0x034A por Stack Write.
 *
 * Nao juntar num unico frame de oito dados: a sequencia pede oito comandos.
 */
static BMS_Status_t sync_dll_writes(BMS_Handle_t *h)
{
    const uint8_t zero = 0U;
    for (uint16_t reg = REG_ECC_DATA1; reg <= REG_ECC_DATA8; ++reg)
    {
        BMS_Status_t st = BMS_WriteStack(h, reg, &zero, 1U);
        if (st != BMS_OK)
        {
            return st;
        }
    }
    return BMS_OK;
}

/**
 * @brief Completa a sincronizacao DLL com oito leituras separadas.
 *
 * Para que serve: confirmar recepcao nos dois slaves depois de atribuir enderecos.
 *
 * Como funciona: le cada registo 0x0343..0x034A por Stack Read; qualquer erro
 * interrompe o arranque e e devolvido ao chamador.
 *
 * O payload e descartado; os frames, enderecos e CRC continuam a ser validados.
 */
static BMS_Status_t sync_dll_reads(BMS_Handle_t *h)
{
    uint8_t buf[BMS_NUM_SLAVES];
    for (uint16_t reg = REG_ECC_DATA1; reg <= REG_ECC_DATA8; ++reg)
    {
        BMS_Status_t st = BMS_ReadStack(h, reg, buf, 1U);
        if (st != BMS_OK)
        {
            return st;
        } /* Nao aceitar 7/8 falhas. */
    }
    return BMS_OK;
}

/**
 * @brief Define qual e o ultimo dispositivo do sentido actual.
 *
 * Para que serve: garantir que existe um unico TOP_STACK na cadeia.
 *
 * Como funciona: coloca os slaves como STACK_DEV; depois coloca o endereco 2
 * como STACK_DEV|TOP_STACK. Em DIR1 esse endereco corresponde a S1 fisico.
 *
 * TOP_STACK e um papel de comunicacao; nao e a bridge nem uma terceira placa.
 */
static BMS_Status_t set_top(BMS_Handle_t *h)
{
    uint8_t data = COMM_CTRL_STACK_DEV;
    BMS_Status_t st = BMS_WriteBroadcast(h, REG_STACK_COMM_CTRL, &data, 1U);
    if (st != BMS_OK)
    {
        return st;
    }
    data = COMM_CTRL_TOP_STACK;
    return BMS_WriteSingleDevice(h, BMS_NUM_SLAVES, REG_STACK_COMM_CTRL, &data, 1U);
}

/* As duas identidades fisicas conservam DIR0/DIR1. Verificar os pares evita
 * confundir os slaves quando a direccao muda. */
/**
 * @brief Confirma os enderecos atribuidos e a resposta da bridge.
 *
 * Para que serve: nao declarar o anel pronto apenas porque as escritas foram enviadas.
 *
 * Como funciona: le o endereco activo ou o par DIR0/DIR1 e compara por identidade
 * fisica. Confirma tambem o endereco 0 e DEV_CONF1 da bridge.
 *
 * @param both true verifica os dois enderecos de cada slave.
 * Espera DEV_CONF1=0x14 porque este driver nao altera esse registo.
 */
static BMS_Status_t verify_addresses(BMS_Handle_t *h, bool both)
{
    uint8_t addr[BMS_NUM_SLAVES * 2U];
    uint16_t reg = both ? REG_BRIDGE_DIR0_ADDR
                        : (h->ring_using_reverse ? REG_BRIDGE_DIR1_ADDR : REG_BRIDGE_DIR0_ADDR);
    uint8_t n = both ? 2U : 1U;
    BMS_Status_t st = BMS_ReadStack(h, reg, addr, n);
    if (st != BMS_OK)
    {
        return st;
    }
    for (uint8_t s = 0U; s < BMS_NUM_SLAVES; ++s)
    {
        if (both)
        {
            if (addr[2U * s] != h->slave[s].address || addr[2U * s + 1U] != h->slave[s].address_rev)
            {
                return BMS_RecordError(h,
                                       BMS_ERR_INIT_FAILED,
                                       "ADDRESS_PAIR",
                                       BMS_GetSlaveAddress(h, s),
                                       reg,
                                       ((uint32_t)h->slave[s].address << 8U) |
                                           h->slave[s].address_rev,
                                       ((uint32_t)addr[2U * s] << 8U) | addr[2U * s + 1U],
                                       0xFFFFU);
            }
        }
        else if (addr[s] != BMS_GetSlaveAddress(h, s))
        {
            return BMS_RecordError(h,
                                   BMS_ERR_INIT_FAILED,
                                   "SLAVE_ADDRESS",
                                   BMS_GetSlaveAddress(h, s),
                                   reg,
                                   BMS_GetSlaveAddress(h, s),
                                   addr[s],
                                   0xFFU);
        }
    }
    uint8_t base[1U];
    st = BMS_ReadSingleDevice(h, BMS_ADDR_BRIDGE, REG_BRIDGE_DEV_CONF1, base, 1U);
    if (st != BMS_OK)
    {
        return st;
    }
    if (base[0] != BRIDGE_DEV_CONF1_DEFAULT)
    {
        return BMS_RecordError(h,
                               BMS_ERR_INIT_FAILED,
                               "BRIDGE_DEV_CONF1",
                               BMS_ADDR_BRIDGE,
                               REG_BRIDGE_DEV_CONF1,
                               BRIDGE_DEV_CONF1_DEFAULT,
                               base[0],
                               0xFFU);
    }
    st = BMS_ReadSingleDevice(h, BMS_ADDR_BRIDGE, REG_BRIDGE_FAULT_MSK, base, 1U);
    if (st != BMS_OK)
    {
        return st;
    }
    if (base[0] != BRIDGE_FAULT_MSK_REQUIRED)
    {
        return BMS_RecordError(h,
                               BMS_ERR_INIT_FAILED,
                               "BRIDGE_FAULT_MASK",
                               BMS_ADDR_BRIDGE,
                               REG_BRIDGE_FAULT_MSK,
                               BRIDGE_FAULT_MSK_REQUIRED,
                               base[0],
                               0xFFU);
    }
    st = BMS_ReadSingleDevice(h,
                              BMS_ADDR_BRIDGE,
                              h->ring_using_reverse ? REG_BRIDGE_DIR1_ADDR : REG_BRIDGE_DIR0_ADDR,
                              base,
                              1U);
    if (st != BMS_OK)
    {
        return st;
    }
    return base[0] == BMS_ADDR_BRIDGE
               ? BMS_OK
               : BMS_RecordError(h,
                                 BMS_ERR_INIT_FAILED,
                                 "BRIDGE_ADDRESS",
                                 BMS_ADDR_BRIDGE,
                                 h->ring_using_reverse ? REG_BRIDGE_DIR1_ADDR
                                                       : REG_BRIDGE_DIR0_ADDR,
                                 BMS_ADDR_BRIDGE,
                                 base[0],
                                 0xFFU);
}

/**
 * @brief Atribui os enderecos 0, 1 e 2 no sentido seleccionado.
 *
 * Para que serve: dar identificadores unicos a bridge e aos dois slaves.
 *
 * Como funciona: envia ADDR_WR por Broadcast normal, depois tres escritas de
 * endereco; define TOP_STACK, faz as leituras DLL e confirma a atribuicao.
 *
 * O dispositivo consome a primeira escrita de endereco e deixa passar as seguintes.
 */
static BMS_Status_t assign_current_direction(BMS_Handle_t *h)
{
    uint8_t data = (h->ring_using_reverse ? CTRL1_DIR_SEL : 0U) | CTRL1_ADDR_WR;
    BMS_Status_t st = BMS_WriteBroadcast(h, REG_BRIDGE_CONTROL1, &data, 1U);
    if (st != BMS_OK)
    {
        return st;
    }
    const uint16_t reg = h->ring_using_reverse ? REG_BRIDGE_DIR1_ADDR : REG_BRIDGE_DIR0_ADDR;
    for (data = 0U; data <= BMS_NUM_SLAVES; ++data)
    {
        st = BMS_WriteBroadcast(h, reg, &data, 1U);
        if (st != BMS_OK)
        {
            return st;
        }
    }
    st = set_top(h);
    if (st != BMS_OK)
    {
        return st;
    }
    st = sync_dll_reads(h);
    if (st != BMS_OK)
    {
        return st;
    }
    return verify_addresses(h, false);
}

/**
 * @brief Comuta os dispositivos para DIR0 ou DIR1.
 *
 * Para que serve: preparar a comunicacao pelo outro lado do anel.
 *
 * Como funciona: limpa o TOP_STACK antigo, muda a bridge, aguarda 100 us, muda
 * os slaves por Broadcast Reverse, aguarda e limpa o topo no novo sentido.
 *
 * Invalida aquisicoes anteriores. O sentido so fica confirmado depois das
 * leituras de verificacao feitas pelo chamador.
 */
static BMS_Status_t change_direction(BMS_Handle_t *h, bool reverse)
{
    if (!valid_handle(h))
    {
        return BMS_ERR_INVALID_PARAM;
    }
    BMS_InhibitOutputs(h);
    h->voltage_valid = h->temperature_valid = h->inverter_voltage_valid = false;
    h->fault_scan_valid = false;
    uint8_t data = COMM_CTRL_STACK_DEV;
    /* Limpar o topo pelo sentido antigo antes da troca. Repetir depois pelo
     * novo sentido, para o caso de uma ligacao anterior estar interrompida. */
    BMS_Status_t st = BMS_WriteBroadcast(h, REG_STACK_COMM_CTRL, &data, 1U);
    if (st != BMS_OK)
    {
        return st;
    }
    data = reverse ? CTRL1_DIR_SEL : 0U;
    h->direction_valid = false;
    st = BMS_WriteSingleDevice(h, BMS_ADDR_BRIDGE, REG_BRIDGE_CONTROL1, &data, 1U);
    if (st != BMS_OK)
    {
        return st;
    }
    h->ring_using_reverse = reverse; /* Ultimo sentido COMANDADO; ainda nao confirmado. */
    BMS_DelayUs(h, DELAY_DIR_SEL_SWITCH_US);
    st = BMS_WriteBroadcastReverse(h, REG_BRIDGE_CONTROL1, INIT_BASE_BCAST_REV_WRITE, &data, 1U);
    if (st != BMS_OK)
    {
        return st;
    }
    BMS_DelayUs(h, DELAY_DIR_SEL_SWITCH_US);
    data = COMM_CTRL_STACK_DEV;
    st = BMS_WriteBroadcast(h, REG_STACK_COMM_CTRL, &data, 1U);
    if (!h->timer_valid)
    {
        return BMS_ERR_PLATFORM;
    }
    return st;
}

/**
 * @brief Selecciona e verifica um caminho completo pelos dois slaves.
 *
 * Para que serve: mudar o sentido sem trocar as identidades fisicas S1/S2.
 *
 * Como funciona: aplica a mudanca necessaria, define topo, sincroniza leituras
 * e confirma ambos os enderecos.
 *
 * Nao implementa operacao com dois segmentos separados por uma rotura central.
 */
BMS_Status_t BMS_SelectDirection(BMS_Handle_t *h, bool reverse)
{
    if (!valid_handle(h) || !h->addresses_valid)
    {
        return BMS_ERR_NOT_READY;
    }
    BMS_Status_t st = BMS_OK;
    if (!h->direction_valid || h->ring_using_reverse != reverse)
    {
        st = change_direction(h, reverse);
    }
    if (st == BMS_OK)
    {
        st = set_top(h);
    }
    if (st == BMS_OK)
    {
        st = sync_dll_reads(h);
    }
    if (st == BMS_OK)
    {
        st = verify_addresses(h, true);
    }
    h->direction_valid = (st == BMS_OK);
    if (st != BMS_OK)
    {
        h->ring_intact = false;
        BMS_InhibitOutputs(h);
    }
    return st;
}

/**
 * @brief Acorda e endereca a rede nos dois sentidos do anel.
 *
 * Para que serve: preparar uma bridge e dois slaves para leituras dirigidas.
 *
 * Como funciona: dois WAKE da bridge -> SEND_WAKE -> DLL/enderecos DIR0 ->
 * DLL/enderecos DIR1 -> regresso a DIR0 -> limpeza dos avisos de arranque.
 *
 * init_step indica a etapa actual. ring_intact so fica true depois de
 * verificar os dois caminhos. Nao e um sensor continuo de todos os cabos.
 */
BMS_Status_t BMS_AutoAddressing(BMS_Handle_t *h)
{
    if (!valid_handle(h))
    {
        return BMS_ERR_INVALID_PARAM;
    }
    BMS_InhibitOutputs(h);
    h->addresses_valid = h->direction_valid = h->ring_intact = false;
    h->config_verified = h->post_passed = false;
    h->diagnostics_confirmed = h->filter_primed = h->soc_valid = false;
    h->ring_using_reverse = false;
    h->voltage_valid = h->temperature_valid = h->inverter_voltage_valid = false;
    h->fault_scan_valid = false;
    for (uint8_t s = 0U; s < BMS_NUM_SLAVES; ++s)
    {
        h->slave[s].comm_ok = false;
        h->slave[s].voltage_valid = h->slave[s].temperature_valid = false;
        h->slave[s].config_crc_valid = false;
        h->slave[s].init_pwr_valid = h->slave[s].init_pwr_cleared = false;
        memset(h->slave[s].init_pwr_raw, 0, sizeof(h->slave[s].init_pwr_raw));
        h->slave[s].address = (uint8_t)(s + 1U);
        h->slave[s].address_rev = (uint8_t)(BMS_NUM_SLAVES - s);
    }
    /* Dois WAKE cobrem tambem o caso especial de shutdown anterior por ping
     * da bridge (TI-600 p.12). Nao representam HW_RESET de toda a stack. */
    h->init_step = "WAKE_BRIDGE";
    BMS_Status_t st = pulse_tx(h, DELAY_WAKE_PULSE_US, 3000U);
    if (st != BMS_OK)
    {
        goto failed;
    }
    BMS_DelayMs(DELAY_OSC_STAB_MS);
    st = pulse_tx(h, DELAY_WAKE_PULSE_US, 3000U);
    if (st != BMS_OK)
    {
        goto failed;
    }
    BMS_DelayMs(DELAY_OSC_STAB_MS);
    h->init_step = "WAKE_STACK";
    uint8_t data = CTRL1_SEND_WAKE;
    st = BMS_WriteSingleDevice(h, BMS_ADDR_BRIDGE, REG_BRIDGE_CONTROL1, &data, 1U);
    if (st != BMS_OK)
    {
        goto failed;
    }
    BMS_DelayMs(DELAY_WAKE_PROPAGATION_MS);
    h->init_step = "DLL_DIR0";
    st = sync_dll_writes(h);
    if (st != BMS_OK)
    {
        goto failed;
    }
    h->init_step = "ADDRESS_DIR0";
    st = assign_current_direction(h);
    if (st != BMS_OK)
    {
        goto failed;
    }
    h->direction_valid = true;
    BMS_IWDG_Refresh(); /* Progresso confirmado, nunca dentro de uma espera bloqueada. */

    h->init_step = "SWITCH_DIR1";
    st = change_direction(h, true);
    if (st != BMS_OK)
    {
        goto failed;
    }
    h->init_step = "DLL_DIR1";
    st = sync_dll_writes(h);
    if (st != BMS_OK)
    {
        goto failed;
    }
    h->init_step = "ADDRESS_DIR1";
    st = assign_current_direction(h);
    if (st != BMS_OK)
    {
        goto failed;
    }
    st = verify_addresses(h, true);
    if (st != BMS_OK)
    {
        goto failed;
    }
    h->addresses_valid = h->direction_valid = true;
    h->init_step = "RETURN_DIR0";
    st = BMS_SelectDirection(h, false);
    if (st != BMS_OK)
    {
        goto failed;
    }

    /* Limpar apenas as categorias esperadas no arranque/sincronizacao.
     * Nao eliminar silenciosamente uma falha de alimentacao/OTP real. */
    data = SLAVE_FAULT_RST1_STARTUP; /* RST_SYS: inclui DRST do arranque. */
    st = BMS_WriteStack(h, REG_FAULT_RST1, &data, 1U);
    if (st != BMS_OK)
    {
        goto failed;
    }
    data = SLAVE_FAULT_RST2_STARTUP; /* COMM1/2 e indicadores COMM3. */
    st = BMS_WriteStack(h, REG_FAULT_RST2, &data, 1U);
    if (st != BMS_OK)
    {
        goto failed;
    }
    data = BRIDGE_FAULT_RST_STARTUP; /* Preservar REG/PWR da bridge. */
    st = BMS_WriteSingleDevice(h, BMS_ADDR_BRIDGE, REG_BRIDGE_FAULT_RST, &data, 1U);
    if (st != BMS_OK)
    {
        goto failed;
    }
    h->ring_intact = true; /* Ambos os sentidos foram verificados nesta sequencia. */
    return BMS_OK;
failed:
    h->addresses_valid = h->direction_valid = h->ring_intact = false;
    h->fault_flags |= BMS_FaultForStatus(st);
    BMS_EmergencyShutdown(h);
    return st; /* Nao esconder TIMEOUT/CRC/FRAME/PLATFORM como INIT_FAILED. */
}

/**
 * @brief Escreve um byte e confirma os bits persistentes por leitura.
 *
 * Para que serve: detectar uma configuracao que nao foi aceite, mesmo com TX concluido.
 *
 * Como funciona: compara (readback & mask) com (value & mask). A mascara ignora
 * bits GO que voltam automaticamente a zero.
 *
 * Nao usar a mascara para ignorar bits de configuracao cuja verificacao e necessaria.
 */
static BMS_Status_t
write_verify(BMS_Handle_t *h, uint8_t dev, uint16_t reg, uint8_t value, uint8_t mask)
{
    uint8_t readback;
    BMS_Status_t st = BMS_WriteSingleDevice(h, dev, reg, &value, 1U);
    if (st != BMS_OK)
    {
        return st;
    }
    st = BMS_ReadSingleDevice(h, dev, reg, &readback, 1U);
    if (st != BMS_OK)
    {
        return st;
    }
    if ((readback & mask) != (value & mask))
    {
        return BMS_RecordError(
            h, BMS_ERR_INIT_FAILED, "WRITE_READBACK", dev, reg, value, readback, mask);
    }
    return BMS_OK;
}

/**
 * @brief Espera que os bits de estado pedidos estejam todos a um.
 *
 * Para que serve: aguardar conclusao do ADC, comparadores ou ciclo CRC interno.
 *
 * Como funciona: repete Single Read com limite temporal e de tentativas; sai
 * ao primeiro erro de comunicacao.
 *
 * @return BMS_OK se a mascara ficou activa; TIMEOUT se nao concluiu no prazo.
 */
static BMS_Status_t
wait_mask(BMS_Handle_t *h, uint8_t dev, uint16_t reg, uint8_t mask, uint32_t timeout_ms)
{
    const uint32_t start = HAL_GetTick();
    uint8_t val = 0U;
    for (uint32_t attempts = 0U; attempts <= timeout_ms; ++attempts)
    {
        BMS_Status_t st = BMS_ReadSingleDevice(h, dev, reg, &val, 1U);
        if (st != BMS_OK)
        {
            return st;
        }
        if ((val & mask) == mask)
        {
            return BMS_OK;
        }
        if ((uint32_t)(HAL_GetTick() - start) >= timeout_ms)
        {
            break;
        }
        BMS_DelayMs(1U);
    }
    return BMS_RecordError(h, BMS_ERR_TIMEOUT, "WAIT_MASK", dev, reg, mask, val, mask);
}

/**
 * @brief Confirma e actualiza o CRC interno dos registos de configuracao.
 *
 * Para que serve: ACTIVE_CELL, OV/UV, ADC_CONF1 e GPIO_CONF sao shadow
 * registers cobertos pelo CRC do BQ. Alterar estes bytes sem actualizar
 * CUST_CRC gera FAULT_OTP.CUST_CRC, mesmo com UART e cabos correctos.
 *
 * Como funciona:
 * 1. Compara TODOS os 54 bytes com a imagem lida antes da configuracao,
 *    actualizada apenas nas posicoes que esta funcao de arranque escreveu.
 * 2. Calcula CRC16 e escreve os dois bytes nos registos VOLATEIS 0x36/0x37.
 * 3. Aguarda um novo ciclo de verificacao interna e compara com 0x50C/0x50D.
 * 4. Recusa falhas OTP diferentes do CUST_CRC transitorio. Nunca mascara OTP.
 *
 * Nao envia chaves de desbloqueio nem comandos de programacao permanente.
 * Fontes: TI-616 7.3.6.4.4.1, 7.5.1, 7.5.4.3.13-16; pp.84,113-115,130-131.
 */
static BMS_Status_t
commit_configuration_crc(BMS_Handle_t *h, uint8_t slave, const uint8_t *expected)
{
    const uint8_t dev = BMS_GetSlaveAddress(h, slave);
    uint8_t actual[BMS_CUST_SHADOW_BYTES], crc_bytes[2], readback[2], stat;
    h->slave[slave].config_crc_valid = false;
    BMS_Status_t st =
        BMS_ReadSingleDevice(h, dev, REG_CUST_SHADOW_START, actual, BMS_CUST_SHADOW_BYTES);
    if (st != BMS_OK)
    {
        return st;
    }
    if (memcmp(actual, expected, sizeof(actual)) != 0)
    {
        uint8_t i = 0U;
        while (i < BMS_CUST_SHADOW_BYTES && actual[i] == expected[i])
        {
            ++i;
        }
        return BMS_RecordError(h,
                               BMS_ERR_INIT_FAILED,
                               "SHADOW_MISMATCH",
                               dev,
                               (uint16_t)(REG_CUST_SHADOW_START + i),
                               i < BMS_CUST_SHADOW_BYTES ? expected[i] : 0U,
                               i < BMS_CUST_SHADOW_BYTES ? actual[i] : 0U,
                               0xFFU);
    }

    /* O CRC e calculado sobre a imagem shadow esperada ja comparada com o readback. */
    const uint16_t crc = BMS_CalculateCRC16(expected, BMS_CUST_SHADOW_BYTES);
    /* CRC reflectido: factory default -> 0xF331. A tabela TI exige
     * CUST_CRC_HI=0x31 e CUST_CRC_LO=0xF3, logo LOW numerico primeiro. */
    crc_bytes[0] = (uint8_t)crc;
    crc_bytes[1] = (uint8_t)(crc >> 8U);
    st = BMS_WriteSingleDevice(h, dev, REG_CUST_CRC_HI, crc_bytes, 2U);
    if (st != BMS_OK)
    {
        return st;
    }
    st = BMS_ReadSingleDevice(h, dev, REG_CUST_CRC_HI, readback, 2U);
    if (st != BMS_OK)
    {
        return st;
    }
    if (memcmp(crc_bytes, readback, sizeof(readback)) != 0)
    {
        return BMS_RecordError(h,
                               BMS_ERR_INIT_FAILED,
                               "CUST_CRC_WRITEBACK",
                               dev,
                               REG_CUST_CRC_HI,
                               ((uint32_t)crc_bytes[0] << 8U) | crc_bytes[1],
                               ((uint32_t)readback[0] << 8U) | readback[1],
                               0xFFFFU);
    }

    /* A primeira leitura limpa um DONE antigo; a seguinte espera por um
     * ciclo completo posterior a actualizacao. DONE e clear-on-read. */
    st = BMS_ReadSingleDevice(h, dev, REG_DEV_STAT, &stat, 1U);
    if (st != BMS_OK)
    {
        return st;
    }
    st = wait_mask(h, dev, REG_DEV_STAT, DEV_STAT_CUST_CRC_DONE, BMS_CUST_CRC_TIMEOUT_MS);
    if (st != BMS_OK)
    {
        return st;
    }
    st = BMS_ReadSingleDevice(h, dev, REG_CUST_CRC_RSLT_HI, readback, 2U);
    if (st != BMS_OK)
    {
        return st;
    }
    if (memcmp(crc_bytes, readback, sizeof(readback)) != 0)
    {
        return BMS_RecordError(h,
                               BMS_ERR_INIT_FAILED,
                               "CUST_CRC_RESULT",
                               dev,
                               REG_CUST_CRC_RSLT_HI,
                               ((uint32_t)crc_bytes[0] << 8U) | crc_bytes[1],
                               ((uint32_t)readback[0] << 8U) | readback[1],
                               0xFFFFU);
    }
    st = BMS_ReadSingleDevice(h, dev, REG_SLAVE_FAULT_OTP, &stat, 1U);
    if (st != BMS_OK)
    {
        return st;
    }
    h->slave[slave].fault_detail_raw[BMS_FD_OTP] = stat;
    h->slave[slave].fault_detail_valid |= SLAVE_SUMMARY_OTP;
    if ((stat & (uint8_t)~SLAVE_FAULT_OTP_CUST_CRC) != 0U)
    {
        h->fault_flags |= BMS_FAULT_HARDWARE;
        return BMS_RecordError(h,
                               BMS_ERR_FAULT_ACTIVE,
                               "OTP_UNEXPECTED",
                               dev,
                               REG_SLAVE_FAULT_OTP,
                               0U,
                               stat,
                               (uint8_t)~SLAVE_FAULT_OTP_CUST_CRC);
    }
    if ((stat & SLAVE_FAULT_OTP_CUST_CRC) != 0U)
    {
        const uint8_t reset_crc = SLAVE_RST2_OTP_CRC;
        st = BMS_WriteSingleDevice(h, dev, REG_FAULT_RST2, &reset_crc, 1U);
        if (st != BMS_OK)
        {
            return st;
        }
        st = BMS_ReadSingleDevice(h, dev, REG_SLAVE_FAULT_OTP, &stat, 1U);
        if (st != BMS_OK)
        {
            return st;
        }
        h->slave[slave].fault_detail_raw[BMS_FD_OTP] = stat;
        if (stat != 0U)
        {
            h->fault_flags |= BMS_FAULT_HARDWARE;
            return BMS_RecordError(h,
                                   BMS_ERR_FAULT_ACTIVE,
                                   "OTP_NOT_CLEARED",
                                   dev,
                                   REG_SLAVE_FAULT_OTP,
                                   0U,
                                   stat,
                                   0xFFU);
        }
    }
    memcpy(h->slave[slave].expected_config_crc, crc_bytes, sizeof(crc_bytes));
    h->slave[slave].config_crc_valid = true;
    return BMS_OK;
}

/* A alteracao intencional de shadow registers pode ter propagado o aviso
 * CUST_CRC como FCOMM. So limpar ESTE aviso, do topo para a bridge, depois
 * de confirmar que nao existe outra causa local. Nao apagar OV/UV/OTP real.
 * E exclusivo do fim da CONFIGURACAO, nao do ciclo normal de monitorizacao. */
/**
 * @brief Limpa apenas a propagacao FCOMM da configuracao acabada de validar.
 *
 * Para que serve: retirar um aviso latched de CRC transitorio sem esconder falhas
 * reais nas celulas, OTP, alimentacao ou comunicacao.
 *
 * Como funciona: percorre do topo para a bridge. Le as causas, recusa categorias
 * inesperadas e limpa FCOMM depois de resolver a origem. PWR so e limpo
 * quando PWR1 contem apenas AVDD_OSC, PWR2=0 e PWR3 apenas AVDDUV_DRST.
 * TI-616 9.5.4.13.25: AVDD_OSC pode surgir em SLEEP->ACTIVE;
 * 9.5.4.13.27: AVDDUV_DRST tambem surge ao acordar de SHUTDOWN/HW Reset.
 * Esta tolerancia e exclusiva do arranque: exige releitura sem PWR.
 *
 * Restrito ao fim da configuracao. O ciclo normal nao limpa erros automaticamente.
 */
static BMS_Status_t finish_configuration_faults(BMS_Handle_t *h)
{
    for (uint8_t step = 0U; step < BMS_NUM_SLAVES; ++step)
    {
        uint8_t dev = (uint8_t)(BMS_NUM_SLAVES - step);
        uint8_t s;
        for (s = 0U; s < BMS_NUM_SLAVES; ++s)
        {
            if (BMS_GetSlaveAddress(h, s) == dev)
            {
                break;
            }
        }
        if (s == BMS_NUM_SLAVES)
        {
            return BMS_ERR_NOT_READY;
        }
        BMS_SlaveData_t *sl = &h->slave[s];
        h->init_step = "FINISH_CONFIGURATION_FAULTS";
        uint8_t summary, comm[3], otp;
        BMS_Status_t st = BMS_ReadSingleDevice(h, dev, REG_SLAVE_FAULT_OTP, &otp, 1U);
        if (st != BMS_OK)
        {
            return st;
        }
        sl->fault_detail_raw[BMS_FD_OTP] = otp;
        sl->fault_detail_valid |= SLAVE_SUMMARY_OTP;
        st = BMS_ReadSingleDevice(h, dev, REG_SLAVE_FAULT_SUMMARY, &summary, 1U);
        if (st != BMS_OK)
        {
            return st;
        }
        sl->fault_summary_raw[0] = summary;
        sl->fault_summary_valid = true;
        st = BMS_ReadSingleDevice(h, dev, REG_SLAVE_FAULT_COMM1, comm, 3U);
        if (st != BMS_OK)
        {
            return st;
        }
        memcpy(&sl->fault_summary_raw[1], comm, sizeof(comm));
        sl->fault_comm_valid = true;
        if ((summary & SLAVE_SUMMARY_PWR) != 0U)
        {
            st = BMS_ReadSingleDevice(
                h, dev, REG_SLAVE_FAULT_PWR1, &sl->fault_detail_raw[BMS_FD_PWR1], 3U);
            if (st != BMS_OK)
            {
                return st;
            }
            sl->fault_detail_valid |= SLAVE_SUMMARY_PWR;
            memcpy(sl->init_pwr_raw, &sl->fault_detail_raw[BMS_FD_PWR1], sizeof(sl->init_pwr_raw));
            sl->init_pwr_valid = true;
            sl->init_pwr_cleared = false;
            if ((sl->init_pwr_raw[0] & (uint8_t)~SLAVE_PWR1_AVDD_OSC) != 0U ||
                sl->init_pwr_raw[1] != 0U ||
                (sl->init_pwr_raw[2] & (uint8_t)~SLAVE_PWR3_AVDDUV_DRST) != 0U)
            {
                const uint8_t allowed[3] = {SLAVE_PWR1_AVDD_OSC, 0U, SLAVE_PWR3_AVDDUV_DRST};
                for (uint8_t i = 0U; i < 3U; ++i)
                {
                    if ((sl->init_pwr_raw[i] & (uint8_t)~allowed[i]) != 0U)
                    {
                        return BMS_RecordError(h,
                                               BMS_ERR_FAULT_ACTIVE,
                                               "WAKE_POWER_UNEXPECTED",
                                               dev,
                                               (uint16_t)(REG_SLAVE_FAULT_PWR1 + i),
                                               allowed[i],
                                               sl->init_pwr_raw[i],
                                               (uint8_t)~allowed[i]);
                    }
                }
            }
        }
        if (otp != 0U || (summary & (uint8_t)~(SLAVE_SUMMARY_COMM | SLAVE_SUMMARY_PWR)) != 0U ||
            comm[0] != 0U || comm[1] != 0U || (comm[2] & (uint8_t)~SLAVE_FAULT_COMM3_FCOMM) != 0U)
        {
            return BMS_RecordError(h,
                                   BMS_ERR_FAULT_ACTIVE,
                                   "STARTUP_SLAVE_FAULT",
                                   dev,
                                   REG_SLAVE_FAULT_SUMMARY,
                                   0U,
                                   summary,
                                   0xFFU);
        }
        if ((summary & SLAVE_SUMMARY_PWR) != 0U)
        {
            h->init_step = "CLEAR_WAKE_POWER_FAULTS";
            const uint8_t clear = SLAVE_RST1_PWR;
            st = BMS_WriteSingleDevice(h, dev, REG_FAULT_RST1, &clear, 1U);
            if (st != BMS_OK)
            {
                return st;
            }
            st = BMS_ReadSingleDevice(h, dev, REG_SLAVE_FAULT_SUMMARY, &summary, 1U);
            if (st != BMS_OK)
            {
                return st;
            }
            sl->fault_summary_raw[0] = summary;
            if ((summary & SLAVE_SUMMARY_PWR) != 0U)
            {
                return BMS_RecordError(h,
                                       BMS_ERR_FAULT_ACTIVE,
                                       "WAKE_POWER_NOT_CLEARED",
                                       dev,
                                       REG_SLAVE_FAULT_SUMMARY,
                                       0U,
                                       summary,
                                       SLAVE_SUMMARY_PWR);
            }
            sl->init_pwr_cleared = true;
            h->init_step = "FINISH_CONFIGURATION_FAULTS";
        }
        if ((comm[2] & SLAVE_FAULT_COMM3_FCOMM) != 0U)
        {
            const uint8_t clear = SLAVE_RST2_FCOMM;
            st = BMS_WriteSingleDevice(h, dev, REG_FAULT_RST2, &clear, 1U);
            if (st != BMS_OK)
            {
                return st;
            }
        }
        st = BMS_ReadSingleDevice(h, dev, REG_SLAVE_FAULT_SUMMARY, &summary, 1U);
        if (st != BMS_OK)
        {
            return st;
        }
        sl->fault_summary_raw[0] = summary;
        if (summary != 0U)
        {
            return BMS_RecordError(h,
                                   BMS_ERR_FAULT_ACTIVE,
                                   "STARTUP_FAULT_REMAINS",
                                   dev,
                                   REG_SLAVE_FAULT_SUMMARY,
                                   0U,
                                   summary,
                                   0xFFU);
        }
    }
    uint8_t bridge[6];
    BMS_Status_t st =
        BMS_ReadSingleDevice(h, BMS_ADDR_BRIDGE, REG_BRIDGE_FAULT_SUMMARY, bridge, sizeof(bridge));
    if (st != BMS_OK)
    {
        return st;
    }
    memcpy(h->bridge_fault_raw, bridge, sizeof(bridge));
    h->bridge_fault_valid = true;
    if ((bridge[0] & (uint8_t)~BRIDGE_SUMMARY_COMM) != 0U || bridge[1] != 0U || bridge[2] != 0U ||
        bridge[3] != 0U || (bridge[4] & (uint8_t)~BRIDGE_FAULT_COMM1_FCOMM) != 0U ||
        bridge[5] != 0U)
    {
        return BMS_RecordError(h,
                               BMS_ERR_FAULT_ACTIVE,
                               "STARTUP_BRIDGE_FAULT",
                               BMS_ADDR_BRIDGE,
                               REG_BRIDGE_FAULT_SUMMARY,
                               0U,
                               bridge[0],
                               0xFFU);
    }
    if ((bridge[4] & BRIDGE_FAULT_COMM1_FCOMM) != 0U)
    {
        const uint8_t clear = BRIDGE_RST_FCOMM;
        st = BMS_WriteSingleDevice(h, BMS_ADDR_BRIDGE, REG_BRIDGE_FAULT_RST, &clear, 1U);
        if (st != BMS_OK)
        {
            return st;
        }
    }
    return BMS_OK; /* POST volta a ler as causas e o nivel fisico NFAULT. */
}

/**
 * @brief Confirma os parametros essenciais que os slaves estao a usar.
 *
 * Para que serve: detectar resets, perda de configuracao ou valores diferentes do esperado.
 *
 * Como funciona: le o CRC de referencia, numero de celulas, limites OV/UV,
 * filtro, controlo do ADC e estado dos comparadores em cada slave.
 *
 * Nao reescreve os valores para esconder uma divergencia: devolve erro e inibe.
 */
BMS_Status_t BMS_VerifyConfiguration(BMS_Handle_t *h)
{
    if (!valid_handle(h) || !h->addresses_valid || !h->direction_valid)
    {
        BMS_InhibitOutputs(h);
        return BMS_RecordError(h, BMS_ERR_NOT_READY, "VERIFY_NOT_READY", 0xFFU, 0U, 1U, 0U, 0U);
    }

    static const struct
    {
        uint16_t reg;
        uint8_t value;
    } bridge_cfg[] = {{REG_BRIDGE_DEV_CONF1, BRIDGE_DEV_CONF1_DEFAULT},
                      {REG_BRIDGE_FAULT_MSK, BRIDGE_FAULT_MSK_REQUIRED}};

    for (size_t i = 0U; i < sizeof(bridge_cfg) / sizeof(bridge_cfg[0]); ++i)
    {
        uint8_t val;
        BMS_Status_t st = BMS_ReadSingleDevice(h, BMS_ADDR_BRIDGE, bridge_cfg[i].reg, &val, 1U);
        if (st == BMS_OK && val != bridge_cfg[i].value)
        {
            st = BMS_RecordError(h,
                                 BMS_ERR_INIT_FAILED,
                                 "BRIDGE_CONFIG_MISMATCH",
                                 BMS_ADDR_BRIDGE,
                                 bridge_cfg[i].reg,
                                 bridge_cfg[i].value,
                                 val,
                                 0xFFU);
        }
        if (st != BMS_OK)
        {
            h->config_verified = false;
            h->fault_flags |= BMS_FaultForStatus(st);
            BMS_EmergencyShutdown(h);
            return st;
        }
    }

    static const struct
    {
        uint16_t reg;
        uint8_t value, mask;
    } cfg[] = {{REG_ACTIVE_CELL, ACTIVE_CELL_15S, 0x0FU},
               {REG_OV_THRESH, OV_THRESH_VAL, 0x3FU},
               {REG_UV_THRESH, UV_THRESH_VAL, 0x3FU},
               {REG_ADC_CONF1, ADC_CONF1_26HZ, 0xFFU},
               {REG_GPIO_CONF1, GPIO_CONF_ADC_ONLY, 0xFFU},
               {REG_GPIO_CONF2, GPIO_CONF_ADC_ONLY, 0x3FU},
               {REG_SLAVE_FAULT_MSK1, 0U, 0xFFU},
               {REG_SLAVE_FAULT_MSK2, 0U, SLAVE_FAULT_MSK2_USED_BITS},
               {REG_CONTROL2, CONTROL2_TSREF_EN, 0x01U},
               {REG_ADC_CTRL1, 0x0AU, 0x1BU}, /* MAIN_GO le-se zero. */
               {REG_OVUV_CTRL, 0x01U, 0xFBU}, /* OVUV_GO le-se zero. */
               {REG_DEV_STAT, DEV_STAT_MAIN_RUN | DEV_STAT_OVUV_RUN, 0x09U}};

    for (uint8_t s = 0U; s < BMS_NUM_SLAVES; ++s)
    {
        uint8_t dev = BMS_GetSlaveAddress(h, s);
        if (!h->slave[s].config_crc_valid)
        {
            h->config_verified = false;
            h->fault_flags |= BMS_FAULT_STALE_DATA;
            BMS_EmergencyShutdown(h);
            return BMS_RecordError(
                h, BMS_ERR_NOT_READY, "CONFIG_CRC_UNAVAILABLE", dev, REG_CUST_CRC_HI, 1U, 0U, 0U);
        }
        uint8_t crc_read[2U];
        BMS_Status_t crc_status = BMS_ReadSingleDevice(h, dev, REG_CUST_CRC_HI, crc_read, 2U);
        if (crc_status != BMS_OK)
        {
            h->config_verified = false;
            h->fault_flags |= BMS_FaultForStatus(crc_status);
            BMS_EmergencyShutdown(h);
            return BMS_RecordError(
                h, crc_status, "CONFIG_CRC_READ", dev, REG_CUST_CRC_HI, 0U, 0U, 0U);
        }
        if (memcmp(crc_read, h->slave[s].expected_config_crc, 2U) != 0)
        {
            h->config_verified = false;
            h->fault_flags |= BMS_FAULT_CONFIG;
            BMS_EmergencyShutdown(h);
            return BMS_RecordError(h,
                                   BMS_ERR_INIT_FAILED,
                                   "CONFIG_CRC_MISMATCH",
                                   dev,
                                   REG_CUST_CRC_HI,
                                   ((uint32_t)h->slave[s].expected_config_crc[0] << 8U) |
                                       h->slave[s].expected_config_crc[1],
                                   ((uint32_t)crc_read[0] << 8U) | crc_read[1],
                                   0xFFFFU);
        }
        for (size_t i = 0U; i < sizeof(cfg) / sizeof(cfg[0]); ++i)
        {
            uint8_t val;
            BMS_Status_t st = BMS_ReadSingleDevice(h, dev, cfg[i].reg, &val, 1U);
            if (st != BMS_OK)
            {
                h->config_verified = false;
                h->fault_flags |= BMS_FaultForStatus(st);
                BMS_EmergencyShutdown(h);
                return BMS_RecordError(
                    h, st, "CONFIG_REGISTER_READ", dev, cfg[i].reg, cfg[i].value, 0U, cfg[i].mask);
            }
            if ((val & cfg[i].mask) != (cfg[i].value & cfg[i].mask))
            {
                h->config_verified = false;
                h->fault_flags |= BMS_FAULT_CONFIG;
                BMS_EmergencyShutdown(h);
                return BMS_RecordError(h,
                                       BMS_ERR_INIT_FAILED,
                                       "CONFIG_REGISTER_MISMATCH",
                                       dev,
                                       cfg[i].reg,
                                       cfg[i].value,
                                       val,
                                       cfg[i].mask);
            }
        }
    }
    h->config_verified = true;
    h->last_config_check_ms = HAL_GetTick();
    return BMS_OK;
}

/**
 * @brief Configura os dois BQ79616 para 15S e monitorizacao de tensao.
 *
 * Para que serve: colocar ambos os slaves no mesmo modo de aquisicao e proteccao OV/UV.
 *
 * Como funciona: para o balanceamento; le a imagem shadow; para/reinicializa o
 * ADC; escreve e confirma 15S, OV/UV, filtro e GPIOs de aquisicao auxiliar;
 * actualiza CRC shadow; arranca ADC/comparadores e verifica. Os restantes
 * bytes shadow sao preservados na imagem usada para calcular o CRC.
 *
 * Os limiares vem das macros do perfil no header. Configurar GPIO/TSREF nao
 * activa as chamadas NTC/HV comentadas na aplicacao. A configuracao e volatil:
 * nao ha programacao permanente OTP.
 */
BMS_Status_t BMS_ConfigureSlaves(BMS_Handle_t *h)
{
    if (!valid_handle(h) || !h->addresses_valid || !h->direction_valid)
    {
        return BMS_ERR_NOT_READY;
    }
    BMS_InhibitOutputs(h);
    h->config_verified = h->post_passed = false;
    h->diagnostics_confirmed = h->filter_primed = h->soc_valid = false;
    h->voltage_valid = h->temperature_valid = h->inverter_voltage_valid = false;
    BMS_Status_t st = BMS_StopAllBalancing(h);
    if (st != BMS_OK)
    {
        return st;
    }
    for (uint8_t s = 0U; s < BMS_NUM_SLAVES; ++s)
    {
        uint8_t dev = BMS_GetSlaveAddress(h, s), stat, all_adc[72U];
        uint8_t expected_shadow[BMS_CUST_SHADOW_BYTES];
        h->init_step = "CONFIGURE_SLAVE";
        h->slave[s].config_crc_valid = false;
        st = BMS_ReadSingleDevice(
            h, dev, REG_CUST_SHADOW_START, expected_shadow, BMS_CUST_SHADOW_BYTES);
        if (st != BMS_OK)
        {
            goto failed;
        }
        for (uint8_t i = 0U; i < 2U; ++i)
        {
            const uint16_t reg = (uint16_t)(REG_SLAVE_FAULT_MSK1 + i);
            const uint8_t mask = i == 0U ? 0xFFU : SLAVE_FAULT_MSK2_USED_BITS;
            if ((expected_shadow[reg] & mask) != 0U)
            {
                st = BMS_RecordError(h,
                                     BMS_ERR_INIT_FAILED,
                                     "SLAVE_FAULT_MASK",
                                     dev,
                                     reg,
                                     0U,
                                     expected_shadow[reg],
                                     mask);
                goto failed;
            }
        }
        expected_shadow[REG_ACTIVE_CELL] = ACTIVE_CELL_15S;
        expected_shadow[REG_OV_THRESH] = OV_THRESH_VAL;
        expected_shadow[REG_UV_THRESH] = UV_THRESH_VAL;
        expected_shadow[REG_ADC_CONF1] = ADC_CONF1_26HZ;
        expected_shadow[REG_GPIO_CONF1] = GPIO_CONF_ADC_ONLY;
        expected_shadow[REG_GPIO_CONF2] = GPIO_CONF_ADC_ONLY;
        st = BMS_ReadSingleDevice(h, dev, REG_DEV_STAT, &stat, 1U);
        if (st != BMS_OK)
        {
            goto failed;
        }
        if ((stat & DEV_STAT_MAIN_RUN) != 0U)
        {
            /* Le todos os resultados principais (inclui DIETEMP1 em 0x05AE/F)
             * antes de parar um ADC que ja estava continuo: TI-616 ADC_CTRL1. */
            st = BMS_ReadSingleDevice(h, dev, REG_VCELL16_HI, all_adc, sizeof(all_adc));
            if (st != BMS_OK)
            {
                goto failed;
            }
        }
        st = write_verify(h, dev, REG_ADC_CTRL1, ADC_CTRL1_STOP, 0x1BU);
        if (st != BMS_OK)
        {
            goto failed;
        }
        /* LPF_EN e GO a zero antes do novo arranque (TI-616 p.29). */
        st = write_verify(h, dev, REG_ADC_CTRL1, 0U, 0x1BU);
        if (st != BMS_OK)
        {
            goto failed;
        }
        st = write_verify(h, dev, REG_ACTIVE_CELL, ACTIVE_CELL_15S, 0x0FU);
        if (st != BMS_OK)
        {
            goto failed;
        }
        BMS_DelayMs(DELAY_AFE_SETTLE_MS); /* ANTES de MAIN_GO. */
        st = write_verify(h, dev, REG_OV_THRESH, OV_THRESH_VAL, 0x3FU);
        if (st != BMS_OK)
        {
            goto failed;
        }
        st = write_verify(h, dev, REG_UV_THRESH, UV_THRESH_VAL, 0x3FU);
        if (st != BMS_OK)
        {
            goto failed;
        }
        st = write_verify(h, dev, REG_ADC_CONF1, ADC_CONF1_26HZ, 0xFFU);
        if (st != BMS_OK)
        {
            goto failed;
        }
        st = write_verify(h, dev, REG_GPIO_CONF1, GPIO_CONF_ADC_ONLY, 0xFFU);
        if (st != BMS_OK)
        {
            goto failed;
        }
        st = write_verify(h, dev, REG_GPIO_CONF2, GPIO_CONF_ADC_ONLY, 0x3FU);
        if (st != BMS_OK)
        {
            goto failed;
        }
        st = write_verify(h, dev, REG_CONTROL2, CONTROL2_TSREF_EN, 0x01U);
        if (st != BMS_OK)
        {
            goto failed;
        }
        BMS_DelayMs(DELAY_TSREF_SETTLE_MS);
        h->init_step = "CONFIG_CRC";
        st = commit_configuration_crc(h, s, expected_shadow);
        if (st != BMS_OK)
        {
            goto failed;
        }
        h->init_step = "START_ADC_OVUV";
        st = write_verify(h, dev, REG_OVUV_CTRL, OVUV_CTRL_ENABLE, 0xFBU);
        if (st != BMS_OK)
        {
            goto failed;
        }
        st = write_verify(h, dev, REG_BAL_CTRL1, BAL_CTRL1_DUTY_5S, 0x07U);
        if (st != BMS_OK)
        {
            goto failed;
        }
        st = write_verify(h, dev, REG_ADC_CTRL1, ADC_CTRL1_CONTINUOUS_LPF, 0x1BU);
        if (st != BMS_OK)
        {
            goto failed;
        }
        st = wait_mask(h, dev, REG_ADC_STAT1, ADC_STAT1_MAIN_READY, BMS_ADC_READY_TIMEOUT_MS);
        if (st != BMS_OK)
        {
            goto failed;
        }
        st = wait_mask(
            h, dev, REG_DEV_STAT, DEV_STAT_MAIN_RUN | DEV_STAT_OVUV_RUN, BMS_ADC_READY_TIMEOUT_MS);
        if (st != BMS_OK)
        {
            goto failed;
        }
        BMS_IWDG_Refresh();
    }
    BMS_DelayMs(DELAY_ADC_SETTLE_MS);
    h->init_step = "VERIFY_CONFIGURATION";
    st = BMS_VerifyConfiguration(h);
    if (st != BMS_OK)
    {
        goto failed;
    }
    st = finish_configuration_faults(h);
    if (st != BMS_OK)
    {
        goto failed;
    }
    return BMS_OK;
failed:
    if (st == BMS_ERR_FAULT_ACTIVE)
    {
        uint32_t hardware_faults = 0U;
        BMS_Status_t scan = BMS_ScanHardwareFaults(h, &hardware_faults);
        h->fault_flags |= hardware_faults;
        if (scan != BMS_OK)
        {
            h->fault_flags |= BMS_FaultForStatus(scan);
        }
        if (h->fault_flags == 0U)
        {
            h->fault_flags |= BMS_FAULT_HARDWARE;
        }
    }
    else if (st == BMS_ERR_INIT_FAILED)
    {
        h->fault_flags |= BMS_FAULT_CONFIG;
    }
    else
    {
        h->fault_flags |= BMS_FaultForStatus(st);
    }
    BMS_EmergencyShutdown(h);
    return st;
}

/**
 * @brief Verifica TIM2 como base de tempo de um microssegundo.
 *
 * Para que serve: evitar pulsos errados por prescaler/clock/ARR incompativeis.
 *
 * Como funciona: calcula o clock de TIM2 a partir de PCLK1, HCLK e TIMPRE;
 * exige contagem ascendente, ARR de 32 bits e frequencia efectiva de 1 MHz.
 *
 * Nao substitui medir os pulsos no hardware.
 */
static bool timer_configuration_ok(TIM_HandleTypeDef *t)
{
    if (t == NULL || t->Instance != TIM2 || t->Instance->PSC > 0xFFFFU ||
        t->Instance->ARR != 0xFFFFFFFFUL || t->Init.CounterMode != TIM_COUNTERMODE_UP ||
        t->Init.Period != 0xFFFFFFFFUL ||
        (t->Instance->CR1 & (TIM_CR1_DIR | TIM_CR1_CMS | TIM_CR1_OPM)) != 0U ||
        (t->Instance->SMCR & (TIM_SMCR_SMS | TIM_SMCR_ECE)) != 0U)
    {
        return false;
    }
    uint32_t pclk = HAL_RCC_GetPCLK1Freq(), hclk = HAL_RCC_GetHCLKFreq();
    if (pclk == 0U || hclk < pclk)
    {
        return false;
    }
    uint32_t div = hclk / pclk;
    uint32_t timer_clk = div == 1U ? pclk : 2U * pclk;
#if defined(RCC_DCKCFGR_TIMPRE)
    if ((RCC->DCKCFGR & RCC_DCKCFGR_TIMPRE) != 0U)
    {
        timer_clk = div <= 4U ? hclk : 4U * pclk;
    }
#endif
    return timer_clk % (t->Instance->PSC + 1U) == 0U &&
           timer_clk / (t->Instance->PSC + 1U) == 1000000UL;
}

/**
 * @brief Termina uma verificacao de plataforma com contexto de diagnostico.
 *
 * Marca PLATFORM, inibe permissoes e coloca o driver em FAULT antes de guardar
 * reason/expected/observed/mask. O chamador garante um handle valido.
 * Devolve BMS_ERR_PLATFORM; nao reinicializa nem corrige o periferico.
 */
static BMS_Status_t platform_init_error(
    BMS_Handle_t *h, const char *reason, uint32_t expected, uint32_t observed, uint32_t mask)
{
    h->fault_flags |= BMS_FAULT_PLATFORM;
    BMS_EmergencyShutdown(h);
    return BMS_RecordError(h, BMS_ERR_PLATFORM, reason, 0xFFU, 0U, expected, observed, mask);
}

/**
 * @brief Liga o driver aos perifericos e executa o arranque dos BQ.
 *
 * Para que serve: inicializar o handle, verificar a plataforma e preparar a aquisicao.
 *
 * Como funciona: aborta DMA anterior quando existe, regista callbacks, valida
 * UART4/TIM2, endereca o anel e configura os slaves.
 *
 * BMS_OK aqui nao equivale ao POST aprovado nem a permissao para o veiculo.
 * Mantem as permissoes inibidas ate as verificacoes seguintes.
 */
BMS_Status_t BMS_Init(BMS_Handle_t *h, UART_HandleTypeDef *u, TIM_HandleTypeDef *t)
{
    if (h == NULL || u == NULL || t == NULL || (g_hbms_irq != NULL && g_hbms_irq != h))
    {
        return BMS_ERR_INVALID_PARAM;
    }
    /* Numa reinicializacao, parar primeiro o DMA antigo: memset nao pode
     * apagar o buffer de destino enquanto o periferico continua a escreve-lo. */
    if (g_hbms_irq == h)
    {
        if (__get_IPSR() != 0U || __get_PRIMASK() != 0U)
        {
            return platform_init_error(
                h, "IRQ_CONTEXT", 0U, __get_IPSR() | __get_PRIMASK(), 0xFFFFFFFFUL);
        }
        if (__atomic_load_n(&h->comm_busy, __ATOMIC_SEQ_CST) != 0U)
        {
            return BMS_RecordError(h, BMS_ERR_COMM, "REINIT_BUSY", 0xFFU, 0U, 0U, 1U, 0U);
        }
        BMS_InhibitOutputs(h);
        HAL_StatusTypeDef abort_status = HAL_UART_AbortReceive(h->huart);
        if (abort_status != HAL_OK)
        {
            h->transport_quarantined = true;
            h->fault_flags |= BMS_FAULT_COMM;
            BMS_EmergencyShutdown(h);
            return BMS_RecordError(
                h, BMS_ERR_COMM, "REINIT_ABORT", 0xFFU, 0U, HAL_OK, abort_status, 0U);
        }
    }
    /* Desligar o destino de callbacks durante a limpeza. */
    uint32_t irq = __get_PRIMASK();
    __disable_irq();
    g_hbms_irq = NULL;
    memset(h, 0, sizeof(*h));
    h->huart = u;
    h->htim_delay = t;
    h->state = BMS_STATE_INITIALIZING;
    h->safety_latched = 1U;
    h->max_temp_c = BMS_INVALID_TEMPERATURE_C;
    h->init_step = "PLATFORM";
    BMS_ClearDebugHistory(h);
    g_hbms_irq = h;
    __set_PRIMASK(irq);
    if (__get_IPSR() != 0U || irq != 0U)
    {
        return platform_init_error(h, "IRQ_CONTEXT", 0U, __get_IPSR() | irq, 0xFFFFFFFFUL);
    }
    if (u->Instance != UART4)
    {
        return platform_init_error(h,
                                   "UART_INSTANCE",
                                   (uint32_t)(uintptr_t)UART4,
                                   (uint32_t)(uintptr_t)u->Instance,
                                   0xFFFFFFFFUL);
    }
    if (u->Init.BaudRate != BMS_UART_BAUDRATE)
    {
        return platform_init_error(
            h, "UART_BAUD", BMS_UART_BAUDRATE, u->Init.BaudRate, 0xFFFFFFFFUL);
    }
    if (u->Init.WordLength != UART_WORDLENGTH_8B)
    {
        return platform_init_error(
            h, "UART_WORDLEN", UART_WORDLENGTH_8B, u->Init.WordLength, 0xFFFFFFFFUL);
    }
    if (u->Init.Parity != UART_PARITY_NONE)
    {
        return platform_init_error(
            h, "UART_PARITY", UART_PARITY_NONE, u->Init.Parity, 0xFFFFFFFFUL);
    }
    if (u->Init.StopBits != UART_STOPBITS_1)
    {
        return platform_init_error(
            h, "UART_STOPBITS", UART_STOPBITS_1, u->Init.StopBits, 0xFFFFFFFFUL);
    }
    if (u->Init.Mode != UART_MODE_TX_RX)
    {
        return platform_init_error(h, "UART_MODE", UART_MODE_TX_RX, u->Init.Mode, 0xFFFFFFFFUL);
    }
    if (u->Init.HwFlowCtl != UART_HWCONTROL_NONE)
    {
        return platform_init_error(
            h, "UART_FLOW", UART_HWCONTROL_NONE, u->Init.HwFlowCtl, 0xFFFFFFFFUL);
    }
    if (u->hdmarx == NULL)
    {
        return platform_init_error(h, "DMA_RX_MISSING", 1U, 0U, 1U);
    }
    if (u->hdmarx->Instance != DMA1_Stream2)
    {
        return platform_init_error(h,
                                   "DMA_RX_STREAM",
                                   (uint32_t)(uintptr_t)DMA1_Stream2,
                                   (uint32_t)(uintptr_t)u->hdmarx->Instance,
                                   0xFFFFFFFFUL);
    }
    if (u->hdmarx->Init.Channel != DMA_CHANNEL_4)
    {
        return platform_init_error(
            h, "DMA_RX_CHANNEL", DMA_CHANNEL_4, u->hdmarx->Init.Channel, 0xFFFFFFFFUL);
    }
    if (u->hdmarx->Init.Direction != DMA_PERIPH_TO_MEMORY)
    {
        return platform_init_error(
            h, "DMA_RX_DIRECTION", DMA_PERIPH_TO_MEMORY, u->hdmarx->Init.Direction, 0xFFFFFFFFUL);
    }
    if (u->hdmarx->Init.PeriphInc != DMA_PINC_DISABLE || u->hdmarx->Init.MemInc != DMA_MINC_ENABLE)
    {
        return platform_init_error(h,
                                   "DMA_RX_INCREMENT",
                                   (DMA_PINC_DISABLE << 16U) | DMA_MINC_ENABLE,
                                   (u->hdmarx->Init.PeriphInc << 16U) | u->hdmarx->Init.MemInc,
                                   0xFFFFFFFFUL);
    }
    if (u->hdmarx->Init.PeriphDataAlignment != DMA_PDATAALIGN_BYTE ||
        u->hdmarx->Init.MemDataAlignment != DMA_MDATAALIGN_BYTE)
    {
        return platform_init_error(h,
                                   "DMA_RX_ALIGNMENT",
                                   (DMA_PDATAALIGN_BYTE << 16U) | DMA_MDATAALIGN_BYTE,
                                   (u->hdmarx->Init.PeriphDataAlignment << 16U) |
                                       u->hdmarx->Init.MemDataAlignment,
                                   0xFFFFFFFFUL);
    }
    if (u->hdmarx->Init.Mode != DMA_NORMAL)
    {
        return platform_init_error(
            h, "DMA_RX_MODE", DMA_NORMAL, u->hdmarx->Init.Mode, 0xFFFFFFFFUL);
    }
    if (!timer_configuration_ok(t))
    {
        return platform_init_error(h,
                                   "TIM2_CONFIG",
                                   1000000UL,
                                   t->Instance != NULL ? t->Instance->PSC : 0xFFFFFFFFUL,
                                   0xFFFFFFFFUL);
    }
    if (HAL_TIM_Base_Start(t) != HAL_OK)
    {
        return platform_init_error(h, "TIM2_START", HAL_OK, HAL_ERROR, 0xFFFFFFFFUL);
    }
    h->timer_valid = true;
    BMS_DelayUs(h, 10U);
    if (!h->timer_valid)
    {
        return BMS_ERR_PLATFORM;
    }
    BMS_IWDG_Refresh();
    BMS_Status_t st = BMS_AutoAddressing(h);
    if (st != BMS_OK)
    {
        return st;
    }
    BMS_IWDG_Refresh();
    st = BMS_ConfigureSlaves(h);
    if (st != BMS_OK)
    {
        return st;
    }
    h->state = BMS_STATE_MONITORING;
    h->init_step = "READY_FOR_POST";
    BMS_InhibitOutputs(h); /* Configurar nao equivale a passar POST ou ter dados. */
    BMS_IWDG_Refresh();
    return BMS_OK;
}

/**
 * @brief Envia o ping SHUTDOWN apenas para a bridge.
 *
 * Para que serve: desligar o BQ79600 e invalidar enderecos/configuracao em RAM.
 *
 * Como funciona: gera o LOW com margem sobre 12.5 ms e espera a transicao.
 *
 * Para desligar tambem os slaves, BMS_EnterSleep envia primeiro SEND_SHUTDOWN.
 * Este ping isolado nao comprova o estado de potencia da stack.
 */
void BMS_SendShutdownPulse(BMS_Handle_t *h)
{
    if (h == NULL)
    {
        return;
    }
    BMS_InhibitOutputs(h);
    if (!valid_handle(h))
    {
        h->state = BMS_STATE_FAULT;
        return;
    }
    BMS_Status_t st = pulse_tx(h, DELAY_SHUTDOWN_PULSE_US, 0U);
    h->addresses_valid = h->direction_valid = h->config_verified = h->post_passed = false;
    h->diagnostics_confirmed = h->filter_primed = h->soc_valid = false;
    h->voltage_valid = h->temperature_valid = h->inverter_voltage_valid = false;
    h->fault_scan_valid = false;
    BMS_DelayMs(5U);
    h->state = st == BMS_OK ? BMS_STATE_SHUTDOWN : BMS_STATE_FAULT;
    if (st != BMS_OK)
    {
        h->fault_flags |= BMS_FAULT_COMM;
    }
}

/* Nome legado mantido. O comportamento pretendido e SHUTDOWN, nao SLEEP.
 * Em modo operacional e preciso confirmar externamente a cadeia fisica
 * aberta; esperar 100 ms sem actuar nada nao seria uma confirmacao. */
/**
 * @brief Pede SHUTDOWN da rede; nome historico mantido por compatibilidade.
 *
 * Para que serve: terminar as comunicacoes e reduzir o consumo depois de parar
 * o balanceamento.
 *
 * Como funciona: confirma as precondicoes existentes, envia o tom SHUTDOWN
 * para a stack e so depois desliga a bridge.
 *
 * Nao altera a logica dos reles. As precondicoes operacionais anteriores
 * mantem-se; esta revisao foca apenas o comportamento dos BQ.
 */
void BMS_EnterSleep(BMS_Handle_t *h)
{
    if (h == NULL)
    {
        return;
    }
    BMS_InhibitOutputs(h);
    if (!valid_handle(h) || !h->addresses_valid || !h->direction_valid)
    {
        h->fault_flags |= BMS_FAULT_COMM;
        h->state = BMS_STATE_FAULT;
        return;
    }
    if (BMS_BENCH_MODE == 0U && !h->physical_safe_confirmed)
    {
        h->state = BMS_STATE_FAULT;
        return;
    }
    if (BMS_StopAllBalancing(h) != BMS_OK)
    {
        h->state = BMS_STATE_FAULT;
        return;
    }
    uint8_t data = (h->ring_using_reverse ? CTRL1_DIR_SEL : 0U) | CTRL1_SEND_SHUTDOWN;
    BMS_Status_t st = BMS_WriteSingleDevice(h, BMS_ADDR_BRIDGE, REG_BRIDGE_CONTROL1, &data, 1U);
    if (st != BMS_OK)
    {
        h->fault_flags |= BMS_FAULT_COMM;
        h->state = BMS_STATE_FAULT;
        return;
    }
    BMS_DelayMs(BMS_SHUTDOWN_STACK_WAIT_MS);
    BMS_SendShutdownPulse(h); /* Pings da bridge, isoladamente, nao desligam a stack. */
}

/**
 * @brief Refresca o watchdog ja iniciado pela configuracao STM32.
 *
 * Para que serve: assinalar que a aplicacao progrediu, sem reconfigurar IWDG.
 *
 * Nao e chamado dentro de esperas infinitas para disfarcar bloqueios.
 */
void BMS_IWDG_Refresh(void)
{
    IWDG_HandleTypeDef watchdog = {0};
    watchdog.Instance = IWDG;
    (void)HAL_IWDG_Refresh(&watchdog);
}

/**
 * @brief Executa as verificacoes basicas antes de monitorizar.
 *
 * Para que serve: confirmar CRC de software, configuracao, tensoes e falhas dos BQ.
 *
 * Como funciona: valida os registos e uma aquisicao completa, le as falhas e
 * confirma NFAULT HIGH sem novo evento concorrente.
 *
 * post_passed nao certifica open-wire nem habilita automaticamente o
 * balanceamento. Nao executa chamadas de NTC/HV que continuam comentadas.
 */
BMS_Status_t BMS_PowerOnSelfTest(BMS_Handle_t *h)
{
    if (!valid_handle(h))
    {
        return BMS_ERR_INVALID_PARAM;
    }
    uint32_t epoch = __atomic_load_n(&h->nfault_epoch, __ATOMIC_SEQ_CST);
    h->init_step = "POST";
    h->post_passed = false;
    BMS_InhibitOutputs(h);
    static const uint8_t crc_test[] = {'1', '2', '3', '4', '5', '6', '7', '8', '9'};
    if (BMS_CalculateCRC16(crc_test, sizeof(crc_test)) != 0x4B37U)
    {
        h->fault_flags |= BMS_FAULT_CRC;
        return BMS_ERR_INIT_FAILED;
    }
    BMS_Status_t st = BMS_VerifyConfiguration(h);
    if (st != BMS_OK)
    {
        return st;
    }
    st = BMS_ReadAllCellVoltages(h);
    if (st != BMS_OK)
    {
        return st;
    }
    st = BMS_CheckProtections(h);
    if (st != BMS_OK)
    {
        return st;
    }
    uint32_t faults = 0U;
    st = BMS_ScanHardwareFaults(h, &faults);
    if (st != BMS_OK || faults != 0U)
    {
        h->fault_flags |= faults;
        if (st != BMS_OK)
        {
            h->fault_flags |= BMS_FaultForStatus(st);
        }
        BMS_EmergencyShutdown(h);
        return st == BMS_OK ? BMS_ERR_FAULT_ACTIVE : st;
    }
    uint32_t irq = __get_PRIMASK();
    __disable_irq();
    bool passed = h->fault_flags == 0U &&
                  HAL_GPIO_ReadPin(BMS_NFAULT_PORT, BMS_NFAULT_PIN) == GPIO_PIN_SET &&
                  __HAL_GPIO_EXTI_GET_IT(BMS_NFAULT_PIN) == RESET &&
                  epoch == __atomic_load_n(&h->nfault_epoch, __ATOMIC_SEQ_CST);
    if (passed)
    {
        /* Reconhece eventos de arranque apenas apos ler todas as falhas,
         * confirmar o nivel e garantir que nao chegou um evento novo. */
        __atomic_store_n(&h->nfault_pending, 0U, __ATOMIC_SEQ_CST);
        __atomic_store_n(&h->safety_latched, 0U, __ATOMIC_SEQ_CST);
        h->post_passed = true;
    }
    __set_PRIMASK(irq);
    if (!passed)
    {
        h->fault_flags |= BMS_FAULT_HARDWARE;
        BMS_EmergencyShutdown(h);
        return BMS_RecordError(h,
                               BMS_ERR_FAULT_ACTIVE,
                               "POST_NFAULT_OR_EVENT",
                               BMS_ADDR_BRIDGE,
                               0U,
                               epoch,
                               __atomic_load_n(&h->nfault_epoch, __ATOMIC_SEQ_CST),
                               0U);
    }
    h->init_step = "MONITORING";
    BMS_UpdateHardwareInterlocks(h);
    return BMS_OK;
}
