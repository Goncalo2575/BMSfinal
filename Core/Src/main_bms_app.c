/**
 * @file main_bms_app.c
 * @brief Aplicacao de bancada. Chamadas antes comentadas permanecem comentadas.
 * A inibicao do driver e LOGICA; esta versao nao passa a accionar reles.
 */
#include "bq796xx_bms.h"
#include "bms_master_comm.h"
#include "bms_relays.h"
#include <stdio.h>
#include <stdbool.h>

#if defined(__GNUC__) || defined(__clang__)
#define BMS_APP_UNUSED __attribute__((unused))
#else
#define BMS_APP_UNUSED
#endif

/* =========================================================================
 * VARIÁVEIS GLOBAIS DA APLICAÇÃO
 * ========================================================================= */

static BMS_Handle_t g_bms;
BMS_MasterComm_t g_master_comm;

extern UART_HandleTypeDef huart4; /* UART4 — BQ79600 bridge (PA0/PA1) */
extern UART_HandleTypeDef huart2; /* USART2 (PA2/PA3) — debug/telemetria TX-only */
extern TIM_HandleTypeDef htim2;   /* TIM2  — Delay µs (contador livre) */

/* BMS_TASK_PERIOD_MS definido no header comum. */

/* =========================================================================
 * FUNÇÕES DE APOIO À APLICAÇÃO
 * ========================================================================= */

/**
 * @brief Aplica a decisao logica de contactor a partir do estado do BMS.
 *
 * Com NFAULT pendente nao tenta fechar. Em monitorizacao/balanceamento sem
 * falhas e fora de carregamento, pede o fecho logico atraves dos interlocks;
 * nas restantes condicoes pede abertura logica. A chamada esta comentada no
 * super-loop de bancada. Nao e um comando directo para um rele GPIO.
 */
static void BMS_APP_UNUSED BMS_ContactorControl(BMS_Handle_t *hbms)
{
    if (__atomic_load_n(&hbms->nfault_pending, __ATOMIC_SEQ_CST) != 0U)
    {
        return;
    }

    bool charging = (BMS_Relays_GetState() == BMS_RLY_CHARGING);

    if (!charging &&
        ((hbms->state == BMS_STATE_MONITORING) || (hbms->state == BMS_STATE_BALANCING)) &&
        (hbms->fault_flags == BMS_FAULT_NONE))
    {
        if (!hbms->contactor_closed)
        {
            BMS_ContactorClose(hbms); /* Barreiras internas reforçam aqui */
        }
    }
    else
    {
        if (hbms->contactor_closed)
        {
            BMS_ContactorOpen(hbms); /* inclui CHARGING → tração inactiva */
        }
    }
}

/**
 * @brief Delega a limpeza dos latches no driver BQ.
 *
 * Para que serve: manter um ponto unico de tratamento dos enderecos correctos
 * da bridge e dos slaves.
 *
 * Nao confirma a ausencia de falha; a recuperacao volta a ler as causas.
 */
static BMS_Status_t BMS_ClearHardwareFaultLatches(BMS_Handle_t *hbms)
{
    return BMS_ClearFaultLatches(hbms);
}

/**
 * @brief Tenta recuperar apenas falhas transitorias de comunicacao.
 *
 * Para que serve: voltar a adquirir os dois slaves depois de revalidar o transporte.
 *
 * Como funciona: limita tentativas; verifica caminho/configuracao; le tensoes;
 * reavalia limites; limpa latches; confirma hardware e repete o POST.
 *
 * OV/UV, sensores e configuracao nao sao apagados por esta recuperacao
 * automatica. Mantem todas as chamadas ja comentadas sem execucao.
 */
static void BMS_FaultRecoveryAttempt(BMS_Handle_t *hbms, uint32_t *retry_counter)
{
    if (hbms == NULL || retry_counter == NULL)
    {
        return;
    }
    BMS_InhibitOutputs(hbms);
    /* Rearme automatico SO para falhas transitorias de comunicacao.
     * Primeiro classificar; uma falha permanente NAO consome tentativas. */
    const uint32_t recoverable_mask =
        BMS_FAULT_COMM | BMS_FAULT_RING_BREAK | BMS_FAULT_CRC | BMS_FAULT_FRAME | BMS_FAULT_HB_FAIL;
    if ((hbms->fault_flags & ~recoverable_mask) != 0U)
    {
        return;
    }
    if (*retry_counter >= BMS_RECOVERY_MAX_ATTEMPTS)
    {
#if BMS_BENCH_MODE
        /* Bancada: manter FAULT e todo o historico para diagnostico. */
        return;
#else
        BMS_EnterSleep(hbms);
        return;
#endif
    }
    ++*retry_counter;
    if (!hbms->direction_valid || (hbms->fault_flags & BMS_FAULT_RING_BREAK) != 0U)
    {
        if (BMS_RingRecovery(hbms) != BMS_OK)
        {
            return;
        }
    }
    BMS_Status_t verify_status = BMS_VerifyConfiguration(hbms);
    if (verify_status != BMS_OK)
    {
        /* Um timeout pode acontecer sem NFAULT/RING_BREAK. Tentar DIR1
         * apenas para erros de transporte, nunca para configuracao invalida. */
        if (verify_status != BMS_ERR_COMM && verify_status != BMS_ERR_TIMEOUT &&
            verify_status != BMS_ERR_CRC && verify_status != BMS_ERR_FRAME)
        {
            return;
        }
        if (BMS_RingRecovery(hbms) != BMS_OK)
        {
            return;
        }
    }
    if (BMS_ReadAllCellVoltages(hbms) != BMS_OK)
    {
        return;
    }
    //BMS_ReadAllTemperatures(hbms);
    /* Nao activar aqui, por outro caminho, a aquisicao que ficou comentada.
     * Em operacional exige-se que o chamador tenha fornecido dados recentes. */
    if (!BMS_MeasurementsFresh(hbms, BMS_BENCH_MODE == 0U, BMS_BENCH_MODE == 0U))
    {
        return;
    }
    /* Reavaliar TODAS as proteccoes, nao apenas a categoria da falha antiga. */
    (void)BMS_CheckProtections(hbms);
    if ((hbms->fault_flags & ~recoverable_mask) != 0U || hbms->min_cell_mv <= CELL_UV_MV + 50U ||
        hbms->max_cell_mv >= CELL_OV_MV - 50U ||
        (hbms->temperature_valid && hbms->max_temp_c >= (int16_t)(CELL_TEMP_MAX_C - 5U)))
    {
        return;
    }
    /* Conservar a falha anterior caso a revalidacao/POST nao conclua o rearme. */
    uint32_t saved_flags = hbms->fault_flags;
    BMS_Status_t clear_status = BMS_ClearHardwareFaultLatches(hbms);
    if (clear_status != BMS_OK)
    {
        hbms->fault_flags |= BMS_FaultForStatus(clear_status);
        return;
    }
    BMS_DelayMs(10U);
    uint32_t faults = 0U;
    BMS_Status_t scan_status = BMS_ScanHardwareFaults(hbms, &faults);
    if (scan_status != BMS_OK || faults != 0U)
    {
        hbms->fault_flags |= faults | BMS_FaultForStatus(scan_status);
        return;
    }
    if (HAL_GPIO_ReadPin(BMS_NFAULT_PORT, BMS_NFAULT_PIN) != GPIO_PIN_SET)
    {
        return;
    }
    /* Continuamos inibidos ate ao POST. Nao se publica BMS_OK entre etapas. */
    hbms->fault_flags = 0U;
    for (uint8_t s = 0U; s < BMS_NUM_SLAVES; ++s)
    {
        hbms->slave[s].fault_flags = 0U;
        memset(hbms->slave[s].ov_cell, 0, sizeof(hbms->slave[s].ov_cell));
        memset(hbms->slave[s].uv_cell, 0, sizeof(hbms->slave[s].uv_cell));
    }
    hbms->state = BMS_STATE_MONITORING;
    if (BMS_PowerOnSelfTest(hbms) != BMS_OK)
    {
        hbms->fault_flags |= saved_flags;
        BMS_EmergencyShutdown(hbms);
        return;
    }
    *retry_counter = 0U;
}

/* =========================================================================
 * PONTO DE ENTRADA DA APLICAÇÃO BMS
 * ========================================================================= */

/**
 * @brief Ponto de entrada da aplicacao de ensaio dos slaves.
 *
 * Para que serve: orquestrar inicializacao, POST, aquisicao a 100 ms e debug a 1 Hz.
 *
 * Como funciona: usa UART4/TIM2 ja configurados em main.c; inicializa os BQ;
 * entra no super-loop e mostra S1/S2. Em erro de arranque regista etapa,
 * dispositivo, registo e resultado. Em bancada mantem o diagnostico e
 * refresca o watchdog; fora desse modo, init-fail aguarda o reset IWDG.
 *
 * As chamadas comentadas no arranque e no ciclo permanecem desactivadas.
 * slaves_ok indica aquisicao verificada; nao e a permissao global bms_ok.
 */
void BMS_Main(void)
{
    BMS_Status_t status;
    uint32_t fault_cycle_count = 0U;      /* ciclos em FAULT desde a última transição */
    uint32_t fault_retry_count = 0U;      /* tentativas de recovery efectuadas */
    uint32_t last_task_tick;              /* timestamp do último ciclo de 100 ms */
    bool last_contactor_decision = false; /* p/ reporte por evento (FTTI) */
    bool last_bms_ok = false;
    uint32_t last_recovery_tick = 0U;
    uint32_t last_fault_report_tick = 0U;
    uint32_t last_reported_fault_flags = 0xFFFFFFFFUL;
    BMS_State_t last_reported_state = BMS_STATE_UNINITIALIZED;
    (void)last_contactor_decision;
    (void)last_bms_ok;

    /* ------------------------------------------------------------------
     * FASE 1: Inicialização do BMS
     * ------------------------------------------------------------------ */
    /* USART2 ja foi inicializada em main.c antes de BMS_Main. O logger directo
     * nao depende de _write/printf e nunca aceita UART4. */
    BMS_MasterComm_Init(&g_master_comm, &huart2);
    BMS_MasterComm_Log(&g_master_comm,
                       "\r\n[BMS] Firmware %u.%u.%u BENCH=%u thresholds=%u/%umV\r\n",
                       (unsigned)BMS_FW_VERSION_MAJOR,
                       (unsigned)BMS_FW_VERSION_MINOR,
                       (unsigned)BMS_FW_VERSION_PATCH,
                       (unsigned)BMS_BENCH_MODE,
                       (unsigned)CELL_UV_MV,
                       (unsigned)CELL_OV_MV);
    BMS_MasterComm_Log(&g_master_comm,
                       "[BMS] Config: %u slaves x %u cells = %u total\r\n",
                       (unsigned)BMS_NUM_SLAVES,
                       (unsigned)BMS_CELLS_PER_SLAVE,
                       (unsigned)BMS_TOTAL_CELLS);

    /* A inicializacao dos reles abaixo continua desactivada.
     * Nao presumir nenhum estado electrico seguro a partir deste comentario. */

    /*          
    BMS_Relays_Init();
    printf("[BMS] Relays/LED init OK (safe-state)\r\n");
    */

    status = BMS_Init(&g_bms, &huart4, &htim2);
    if (status != BMS_OK)
    {
        /* A tentativa terminou; reservar o orcamento do IWDG para o dump. */
        BMS_IWDG_Refresh();
        BMS_MasterComm_Log(&g_master_comm,
                           "[BMS] INIT FAIL status=%s step=%s last=%s dev=%02X reg=%04X\r\n",
                           BMS_GetStatusString(status),
                           g_bms.init_step ? g_bms.init_step : "UNKNOWN",
                           BMS_GetStatusString(g_bms.last_error_status),
                           (unsigned)g_bms.last_error_device,
                           (unsigned)g_bms.last_error_reg);
        BMS_MasterComm_PrintFaults(&g_master_comm, &g_bms);
        BMS_MasterComm_PrintDiagnostics(&g_master_comm, &g_bms);
#if BMS_BENCH_MODE && BMS_BENCH_HOLD_ON_INIT_FAILURE
        /* Para diagnostico de bancada NAO perder FIRST/LAST por reset de IWDG.
         * Nao se fazem novas transaccoes UART4 neste estado. */
        uint32_t last_print = HAL_GetTick();
        while (1U)
        {
            BMS_IWDG_Refresh();
            if ((uint32_t)(HAL_GetTick() - last_print) >= BMS_DEBUG_REPEAT_MS)
            {
                last_print = HAL_GetTick();
                BMS_MasterComm_Log(&g_master_comm, "[BMS] INIT FAIL mantido para debug\r\n");
                BMS_MasterComm_PrintDiagnostics(&g_master_comm, &g_bms);
            }
            BMS_DelayMs(10U);
        }
#else
        while (1U)
        {
            BMS_DelayMs(200U);
        } /* IWDG provoca nova tentativa. */
#endif
    }

    BMS_MasterComm_Log(&g_master_comm,
                       "[BMS] Initialization OK state=%s ring=%s\r\n",
                       BMS_GetStateString(g_bms.state),
                       g_bms.ring_intact ? "YES" : "NO");

    /* ------------------------------------------------------------------
     * FASE 1b: Inicializar saída de debug/telemetria (USART2, TX-only)
     * ------------------------------------------------------------------ */

    BMS_MasterComm_Log(&g_master_comm,
                       "[BMS] Debug USART2 activo (PA2 TX, baud=%lu)\r\n",
                       (unsigned long)huart2.Init.BaudRate);

    /* ------------------------------------------------------------------
     * FASE 1c: IWDG Watchdog (supervisao de bloqueio) — armado pelo CubeMX (MX_IWDG_Init).
     * ------------------------------------------------------------------ */
    BMS_MasterComm_Log(&g_master_comm,
                       "[BMS] IWDG: requer MX_IWDG_Init; BENCH preserva init-fail para debug\r\n");

    /* ------------------------------------------------------------------
     * FASE 1d: Power-On Self Test (POST)
     * ------------------------------------------------------------------ */
    status = BMS_PowerOnSelfTest(&g_bms);
    if (status != BMS_OK)
    {
        BMS_MasterComm_Log(&g_master_comm,
                           "[BMS] POST FAILED fault=%s (0x%08lX)\r\n",
                           BMS_GetFaultString(g_bms.fault_flags),
                           (unsigned long)g_bms.fault_flags);
        BMS_MasterComm_PrintFaults(&g_master_comm, &g_bms);
        BMS_MasterComm_PrintDiagnostics(&g_master_comm, &g_bms);
        g_bms.state = BMS_STATE_FAULT;
    }
    else
    {
        BMS_MasterComm_Log(&g_master_comm, "[BMS] POST basico OK (bridge/slaves)\r\n");
    }

    /* ------------------------------------------------------------------
     * FASE 2: Decisão lógica inicial de contactor (gating de SoC/telemetria;
     *          actuação física é do módulo de relés).
     * ------------------------------------------------------------------ */
    BMS_DelayMs(100U);
    status = BMS_ReadAllCellVoltages(&g_bms);
    if (status != BMS_OK)
    {
        BMS_EmergencyShutdown(&g_bms);
    }
    else
    {
        (void)BMS_CheckProtections(&g_bms);
    }
    //BMS_ReadAllTemperatures(&g_bms);
    //BMS_ReadInverterVoltage(&g_bms);          /* tensão do bus p/ pré-carga *
    BMS_UpdateHardwareInterlocks(&g_bms); /* calcula bms_ok inicial */

    if ((g_bms.fault_flags == BMS_FAULT_NONE) && (g_bms.post_passed))
    {
        BMS_MasterComm_Log(&g_master_comm,
                           "[BMS] Aquisicao inicial OK; BMS_OK=%u (bench=0 esperado)\r\n",
                           (unsigned)g_bms.bms_ok);
        //BMS_ContactorClose(&g_bms);
    }
    else
    {
        BMS_MasterComm_Log(&g_master_comm,
                           "[BMS] Startup fault=%s (0x%08lX)\r\n",
                           BMS_GetFaultString(g_bms.fault_flags),
                           (unsigned long)g_bms.fault_flags);
    }

    /* Primeira passagem da malha de segurança/relés + reporte inicial */
    //BMS_Relays_Task(&g_bms, HAL_GetTick());
    //BMS_MasterComm_PrintDebug(&g_master_comm, &g_bms);
    slavestesteprint(&g_master_comm, &g_bms);
    last_fault_report_tick = HAL_GetTick();
    last_reported_fault_flags = g_bms.fault_flags;
    last_reported_state = g_bms.state;
    //last_contactor_decision = g_bms.contactor_closed;
    last_bms_ok = g_bms.bms_ok;

    /* ------------------------------------------------------------------
     * FASE 3: Super-loop de operação
     * ------------------------------------------------------------------ */
    //printf("[BMS] Entering monitoring loop. Safety state: %s\r\n", BMS_Relays_GetStateString());

    last_task_tick = HAL_GetTick();

    while (1U)
    {
        /* IWDG: refrescar em TODAS as iterações (independente do tick). */
        BMS_IWDG_Refresh();

        /* Malha de reles mantida comentada: nao ha actuacao fisica aqui. */
        //BMS_Relays_Task(&g_bms, HAL_GetTick());

        /* Inibicao imediata; a cadencia UART e central em BMS_ProcessFaults. */
        BMS_UpdateHardwareInterlocks(&g_bms);
        if (g_bms.state != BMS_STATE_SHUTDOWN && g_bms.state != BMS_STATE_SLEEP &&
            BMS_NfaultIsActive(&g_bms))
        {
            BMS_InhibitOutputs(&g_bms);
            /* Chamada original continua comentada. Numa futura integracao,
             * o corte fisico tem prioridade sobre o diagnostico UART. */
            //BMS_Relays_Task(&g_bms, HAL_GetTick());
            (void)BMS_ProcessFaults(&g_bms);

            //BMS_MasterComm_PrintDebug(&g_master_comm, &g_bms);
            /* Debug imediato apenas se a causa/estado mudou; caso contrario
             * o heartbeat de 1 s mostra o mesmo snapshot sem saturar USART2. */
            uint32_t now_dbg = HAL_GetTick();
            if (g_bms.fault_flags != last_reported_fault_flags ||
                g_bms.state != last_reported_state ||
                (uint32_t)(now_dbg - last_fault_report_tick) >= BMS_DEBUG_REPEAT_MS)
            {
                slavestesteprint(&g_master_comm, &g_bms);
                last_fault_report_tick = now_dbg;
                last_reported_fault_flags = g_bms.fault_flags;
                last_reported_state = g_bms.state;
            }
            //last_contactor_decision = g_bms.contactor_closed;
            last_bms_ok = g_bms.bms_ok;
        }

        /* Cadência de 100 ms via HAL_GetTick (base TIM6). */
        if ((HAL_GetTick() - last_task_tick) < BMS_TASK_PERIOD_MS)
        {
            BMS_DelayMs(1U);
            continue;
        }
        last_task_tick = HAL_GetTick(); /* Nao recuperar atrasos com rajadas de ciclos. */

        /* TAREFA PRINCIPAL DO BMS @ 100 ms */
        status = BMS_Task_100ms(&g_bms);
        if (status != BMS_OK)
        {
            BMS_InhibitOutputs(&g_bms);
        }
        (void)last_bms_ok;

        /* Decisão LÓGICA de contactor (gating de SoC/telemetria) */
        //BMS_ContactorControl(&g_bms);

        /* REPORTE POR EVENTO: se a decisão de contactor ou o BMS_OK mudaram. 
        if ((g_bms.contactor_closed != last_contactor_decision) ||
            (g_bms.bms_ok           != last_bms_ok))
        {
            BMS_MasterComm_PrintDebug(&g_master_comm, &g_bms);
            last_contactor_decision = g_bms.contactor_closed;
            last_bms_ok             = g_bms.bms_ok;
            debug_print_div         = 0U;
        }*/

        /* GESTÃO DE RECOVERY EM ESTADO FAULT */
        if (g_bms.state == BMS_STATE_FAULT)
        {
            if (fault_cycle_count == 0U)
            {
                fault_cycle_count = 1U;
                last_recovery_tick = HAL_GetTick();
            }
            if ((uint32_t)(HAL_GetTick() - last_recovery_tick) >= BMS_RECOVERY_PERIOD_MS &&
                fault_retry_count < BMS_RECOVERY_MAX_ATTEMPTS)
            {
                last_recovery_tick = HAL_GetTick();
                BMS_FaultRecoveryAttempt(&g_bms, &fault_retry_count);
            }
        }
        else
        {
            fault_cycle_count = 0U;
            fault_retry_count = 0U;
        }

        /* Reportar tambem falhas de software sem flanco NFAULT. Cadencia real,
         * independente de quantos ciclos se perderam em UART/recuperacao. */
        if (g_bms.fault_flags != last_reported_fault_flags || g_bms.state != last_reported_state ||
            (uint32_t)(HAL_GetTick() - last_fault_report_tick) >= BMS_DEBUG_REPEAT_MS)
        {
            //BMS_MasterComm_PrintDebug(&g_master_comm, &g_bms);
            slavestesteprint(&g_master_comm, &g_bms);
            last_fault_report_tick = HAL_GetTick();
            last_reported_fault_flags = g_bms.fault_flags;
            last_reported_state = g_bms.state;
        }
    }
}
