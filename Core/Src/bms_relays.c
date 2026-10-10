/**
 * @file    bms_relays.c
 * @brief   BMS - Máquina de estados de segurança + actuação de relés (implementação)
 *
 *  
 *  BMS actual (BQ796xx):
 *    - BMS_OK         <- derivado de hbms->fault_flags + nfault_pending
 *    - tensão do B+  <- hbms->inverter_voltage_mv (GPIO4 / sensor HV)
 *    - tensão do pack <- hbms->pack_voltage_mv (soma das células)
 *    - monitores      <- polling com debounce (sem EXTI; evita conflito com
 *                        a EXTI9_5 do NFAULT)
 *    - atraso de pré-carga (750 ms) NÃO-bloqueante (o IWDG ~500 ms proíbe
 *      HAL_Delay longos no super-loop)
 *
 */

#include "bms_relays.h"

#define RLY_CLOSE(port, pin) HAL_GPIO_WritePin((port), (pin), GPIO_PIN_SET)
#define RLY_OPEN(port, pin)  HAL_GPIO_WritePin((port), (pin), GPIO_PIN_RESET)

/* =========================================================================
 * ESTADO INTERNO
 * ========================================================================= */
static BMS_RelayState_t s_state = BMS_RLY_NOT_SAFE; // estado de segurança actual
static bool s_state_change;
static bool s_pre_charge_enable;
static bool s_dis_charge_enable;
static bool s_charger_relay_enable;
static bool s_bms_relay_open_latched;

static uint32_t s_engaged_tick; /* p/ atraso de pré-carga */

/* --- Debounce dos monitores --- */
typedef struct
{
    GPIO_TypeDef *port;
    uint16_t pin;
    uint8_t last_raw;
    uint8_t stable; /* 1 = activo (pino HIGH) */
    uint32_t t;     /* início da contagem de debounce */
} MonCh_t;

typedef enum
{
    MON_IMD = 0,  /* PB0  */
    MON_TSMS,     /* PC8  */
    MON_ESDB,     /* PC6  */
    MON_ESDB_CHG, /* PB14 */
    MON_CHARGER,  /* PB12 */
    MON_COUNT
} MonId_t;

static MonCh_t s_mon[MON_COUNT];

/* =========================================================================
 * UTILITÁRIOS
 * ========================================================================= */

/**
 * @brief  Inicializa um canal de monitor (porta/pino + estado de arranque)
 */

static void mon_seed(MonId_t id, GPIO_TypeDef *port, uint16_t pin)
{
    s_mon[id].port = port;
    s_mon[id].pin = pin;
    s_mon[id].last_raw = 0U;
    s_mon[id].stable = 0U;
    s_mon[id].t = 0U;
}

/**
 * @brief  Actualiza o estado estável de um monitor com debounce temporal
 *
 * lê o estado de uma entrada digital (como um botão ou um sensor)
 *  e garantir que o sistema só reage a uma mudança de estado (de 0 para 1, ou de 1 para 0) 
 * se essa mudança se mantiver estável por um período definido (BMS_RELAY_DEBOUNCE_MS, que é 100ms),
 *  ignorando assim ruídos elétricos ou ressaltos mecânicos que durem menos tempo.
 */
static void mon_update(MonId_t id, uint32_t now_ms)
{
    MonCh_t *d = &s_mon[id];
    uint8_t raw = (HAL_GPIO_ReadPin(d->port, d->pin) == GPIO_PIN_SET) ? 1U : 0U;

    if (raw != d->stable)
    {
        if (raw != d->last_raw)
        {
            d->last_raw = raw;
            d->t = now_ms; /* nova transição -> reinicia debounce */
        }
        else if ((now_ms - d->t) >= BMS_RELAY_DEBOUNCE_MS)
        {
            d->stable = raw; /* estável durante o tempo de debounce */
        }
    }
    else
    {
        d->last_raw = raw;
    }
}

/* =========================================================================
 * INICIALIZAÇÃO
 * ========================================================================= */

/**
 * @brief Inicializa o estado do modulo e, fora da bancada, os GPIO de reles/LEDs.
 *
 * Em BMS_BENCH_MODE retorna sem configurar ou escrever GPIO, mantendo NOT_SAFE
 * e as permissoes internas de pre-carga/descarga/carregador a false.
 * No ramo operacional configura entradas com pull-down, saidas e debounce;
 * fecha as permissoes BMS/BMS_charge, abre os restantes reles e apaga LEDs.
 * Essa sequencia descreve o codigo legado e depende da integracao electrica.
 * O MX_GPIO_Init do projecto STM32 e independente desta funcao.
 */
void BMS_Relays_Init(void)
{
    /* Mesmo uma chamada acidental nao deve configurar/energizar reles na
     * revisao dedicada aos BQ. A integracao operacional continua separada. */
    if (BMS_BENCH_MODE != 0U)
    {
        s_state = BMS_RLY_NOT_SAFE;
        s_pre_charge_enable = s_dis_charge_enable = s_charger_relay_enable = false;
        return;
    }
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    GPIO_InitTypeDef g = {0};

    /* --- Saídas: relés + LEDs --- */
    g.Mode = GPIO_MODE_OUTPUT_PP;
    g.Pull = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_LOW;

    g.Pin = BMS_PRE_CHARGE_PIN;
    HAL_GPIO_Init(BMS_PRE_CHARGE_PORT, &g);
    g.Pin = BMS_CHARGE_RELAY_PIN;
    HAL_GPIO_Init(BMS_CHARGE_RELAY_PORT, &g);
    g.Pin = BMS_DISCHARGE_PIN;
    HAL_GPIO_Init(BMS_DISCHARGE_PORT, &g);
    g.Pin = BMS_RELAY_PIN;
    HAL_GPIO_Init(BMS_RELAY_PORT, &g);
    g.Pin = BMS_BMSCHARGE_PIN;
    HAL_GPIO_Init(BMS_BMSCHARGE_PORT, &g);
    g.Pin = BMS_LED_GREEN_PIN;
    HAL_GPIO_Init(BMS_LED_GREEN_PORT, &g);
    g.Pin = BMS_LED_RED_PIN;
    HAL_GPIO_Init(BMS_LED_RED_PORT, &g);
    g.Pin = BMS_LED_BLUE_PIN;
    HAL_GPIO_Init(BMS_LED_BLUE_PORT, &g);

    /* --- Entradas: monitores (pull-down fail-safe) --- */
    g.Mode = GPIO_MODE_INPUT;
    g.Pull = GPIO_PULLDOWN;
    g.Pin = BMS_IMD_STATUS_PIN;
    HAL_GPIO_Init(BMS_IMD_STATUS_PORT, &g);
    g.Pin = BMS_TSMS_PIN;
    HAL_GPIO_Init(BMS_TSMS_PORT, &g);
    g.Pin = BMS_ESDB_PIN;
    HAL_GPIO_Init(BMS_ESDB_PORT, &g);
    g.Pin = BMS_ESDB_CHG_PIN;
    HAL_GPIO_Init(BMS_ESDB_CHG_PORT, &g);
    g.Pin = BMS_CHARGER_SIG_PIN;
    HAL_GPIO_Init(BMS_CHARGER_SIG_PORT, &g);

    /* Sequencia operacional legada: fecha permissoes BMS; depende da integracao fisica. */
    RLY_CLOSE(BMS_RELAY_PORT, BMS_RELAY_PIN);              /* BMS relay fechado */
    RLY_CLOSE(BMS_BMSCHARGE_PORT, BMS_BMSCHARGE_PIN);      /* BMS charge fechado */
    RLY_OPEN(BMS_CHARGE_RELAY_PORT, BMS_CHARGE_RELAY_PIN); /* charger relay aberto */
    RLY_OPEN(BMS_PRE_CHARGE_PORT, BMS_PRE_CHARGE_PIN);     /* pré-carga aberto */
    RLY_OPEN(BMS_DISCHARGE_PORT, BMS_DISCHARGE_PIN);       /* descarga aberto */

    RLY_OPEN(BMS_LED_RED_PORT, BMS_LED_RED_PIN);     /* LED vermelho apagado */
    RLY_OPEN(BMS_LED_GREEN_PORT, BMS_LED_GREEN_PIN); /* LED verde apagado */
    RLY_OPEN(BMS_LED_BLUE_PORT, BMS_LED_BLUE_PIN);   /* LED azul apagado */

    /* --- Estado lógico --- */
    s_state = BMS_RLY_NOT_SAFE;
    s_state_change = true;
    s_pre_charge_enable = false;
    s_dis_charge_enable = false;
    s_charger_relay_enable = false;
    s_bms_relay_open_latched = false;
    s_engaged_tick = 0U;

    mon_seed(MON_IMD,
             BMS_IMD_STATUS_PORT,
             BMS_IMD_STATUS_PIN); /* Associa o pino; a leitura so ocorre em mon_update. */
    mon_seed(MON_TSMS, BMS_TSMS_PORT, BMS_TSMS_PIN);
    mon_seed(MON_ESDB, BMS_ESDB_PORT, BMS_ESDB_PIN);
    mon_seed(MON_ESDB_CHG, BMS_ESDB_CHG_PORT, BMS_ESDB_CHG_PIN);
    mon_seed(MON_CHARGER, BMS_CHARGER_SIG_PORT, BMS_CHARGER_SIG_PIN);
}

/* =========================================================================
 * TAREFA PRINCIPAL
 * ========================================================================= */

/**
 * @brief  Tarefa de segurança não-bloqueante: monitores -> estado -> relés/LED
 *
 * Para que serve: é o coração da malha de segurança e da actuação física.
 * Em bancada retorna imediatamente; a chamada no main esta comentada.
 * Na integracao operacional, deve correr em todas as iteracoes para o debounce,
 * o blink do LED e a pré-carga temporizada serem fluidos. Por etapas:
 *  1) amostra+debounce dos 5 monitores;
 *  2) deriva BMS_OK (fault_flags + nfault_pending) e bms_state_active (estado);
 *  3) lê tensões do B+/pack para a lógica de pré-carga/descarga;
 *  4) relé do carregador segue o optoacoplador (charger);
 *  5) BMS_relay + BMS_charge ABREM em falha OU fora de estado activo e ficam em LATCH;
 *  6) calcula esdb_flag e tsms_flag;
 *  7) auto-abertura de descarga/pré-carga por tensão do B+ (excepto bleed em
 *     CHARGING, que se mantém fechado);
 *  8) selecciona SAFE/ENGAGED/CHARGING/NOT_SAFE (NOT_SAFE tem prioridade e
 *     cobre também SLEEP/SHUTDOWN/UNINIT/FAULT);
 *  9) acções de ENTRADA de estado: SAFE/NOT_SAFE = bleed transitório; CHARGING =
 *     bleed CONTÍNUO; ENGAGED = pré-carga;
 * 10) pré-carga adiada do ENGAGED (750 ms, não-bloqueante);
 * 11) conduz o LED cluster a partir do estado.
 *
 */
void BMS_Relays_Task(BMS_Handle_t *hbms, uint32_t now_ms)
{
    if (BMS_BENCH_MODE != 0U)
    {
        return;
    }
    if (hbms == NULL)
    {
        return;
    }

    /* --- 1) Amostragem + debounce dos monitores --- */
    for (uint8_t i = 0U; i < (uint8_t)MON_COUNT; i++)
    {
        mon_update((MonId_t)i, now_ms);
    }

    bool imd_ok = (s_mon[MON_IMD].stable != 0U);
    bool tsms_wd = (s_mon[MON_TSMS].stable != 0U);
    bool esdb_wd = (s_mon[MON_ESDB].stable != 0U);
    bool charger_opto = (s_mon[MON_CHARGER].stable != 0U);

    /* --- 2) BMS_OK (derivado da BMS actual: faults + NFAULT de hardware) --- */
    bool bms_ok = (hbms->fault_flags == 0U) &&
                  (__atomic_load_n(&hbms->nfault_pending, __ATOMIC_SEQ_CST) == 0U);

    /* Estado activo do BMS: só MONITORING/BALANCING têm aquisição a correr.
     * Em SLEEP/SHUTDOWN/UNINIT/FAULT/RING_RECOVERY o sistema NÃO está apto
     * Entra na decisão de relé e de estado de segurança. */
    bool bms_state_active =
        (hbms->state == BMS_STATE_MONITORING) || (hbms->state == BMS_STATE_BALANCING);

    /* --- 3) Tensões (mV) para pré-carga/descarga --- */
    uint32_t bus_mv = hbms->inverter_voltage_mv;  /*tensão do B+*/
    uint32_t pack_mv = hbms->pack_voltage_mv;     /*tensão do pack*/
    bool bus_ge_min = (bus_mv >= BMS_BUS_MIN_MV); /*B+ >= tensão mínima*/
    bool bus_le_min = (bus_mv <= BMS_BUS_MIN_MV); /*B+ <= tensão mínima*/
    uint32_t pre_thr = (pack_mv > 0U) ? (pack_mv * BMS_PRECHARGE_PCT_NUM) / BMS_PRECHARGE_PCT_DEN
                                      : 0U;                 /*calcula limite de pré-carga*/
    bool bus_ge_90 = (pack_mv > 0U) && (bus_mv >= pre_thr); /*B+ >= 90% da tensão do pack*/
    bool bus_lt_90 = (pack_mv > 0U) && (bus_mv < pre_thr);  /*B+ < 90% da tensão do pack*/

    /* --- 4) Relé do CARREGADOR (charger ligado)  --- */
    if (charger_opto)
    {
        // Se o opto do carregador estiver activo, fecha o relé do carregador.
        if (!s_charger_relay_enable)
        {
            RLY_CLOSE(BMS_CHARGE_RELAY_PORT, BMS_CHARGE_RELAY_PIN);
            s_charger_relay_enable = true;
        }
    }
    else
    {
        // Se o opto do carregador estiver inactivo, abre o relé do carregador.
        if (s_charger_relay_enable)
        {
            RLY_OPEN(BMS_CHARGE_RELAY_PORT, BMS_CHARGE_RELAY_PIN);
            s_charger_relay_enable = false;
        }
    }

    /* --- 5) BMS relay + BMS charge: abrir em falha OU fora de estado activo
     * (latch). Estes são os relés de PERMISSÃO do BMS: BMS_relay alimenta o loop
     * de segurança do Line_contactor (drive) e BMS_charge_relay o do
     * charge_contactor, mesma lógica. . !bms_state_active
     * garante relé aberto em SLEEP/SHUTDOWN. */
    if (!bms_ok || !imd_ok || !bms_state_active)
    {
        RLY_OPEN(BMS_RELAY_PORT, BMS_RELAY_PIN);
        RLY_OPEN(BMS_BMSCHARGE_PORT, BMS_BMSCHARGE_PIN);
        s_bms_relay_open_latched = true;
    }
#if (BMS_RELAY_AUTO_RECLOSE != 0)
    else if (s_bms_relay_open_latched)
    {
        RLY_CLOSE(BMS_RELAY_PORT, BMS_RELAY_PIN);
        RLY_CLOSE(BMS_BMSCHARGE_PORT, BMS_BMSCHARGE_PIN);
        s_bms_relay_open_latched = false;
    }
#endif

    /* --- 6) ESDB / TSMS  ---
     * A decisao usa MON_ESDB (PC6). MON_ESDB_CHG e amostrado para o snapshot,
     * mas nao participa nesta expressao; requer revisao na integracao operacional. */
    bool esdb_flag = esdb_wd;
    bool tsms_flag = esdb_flag && tsms_wd;

    /* --- 7) Auto-abertura de descarga / pré-carga por tensão do bus ---
     * Em CHARGING com BMS_BLEED_HOLD_IN_CHARGING=1 o bleed NÃO se auto-abre —
     * mantem-se comandado fechado enquanto durar esse estado.
     * Com =0, a abertura automatica usa o limiar de 5 V, como em SAFE. */
    bool keep_bleed_charging = false;

#if (BMS_BLEED_HOLD_IN_CHARGING)
    keep_bleed_charging = (s_state == BMS_RLY_CHARGING);
#endif

    // Se o bus estiver abaixo do limiar mínimo (5 V), abre a descarga/bleed.
    if (s_dis_charge_enable && bus_le_min && !keep_bleed_charging)
    {
        RLY_OPEN(BMS_DISCHARGE_PORT, BMS_DISCHARGE_PIN);
        s_dis_charge_enable = false;
    }
    // Se o bus estiver acima do limiar máximo (90%), abre a pré-carga.
    if (s_pre_charge_enable && bus_ge_90)
    {
        RLY_OPEN(BMS_PRE_CHARGE_PORT, BMS_PRE_CHARGE_PIN);
        s_pre_charge_enable = false;
    }

    /* --- 8) Selecção de estado --- */
    BMS_RelayState_t ns; //devolve o estado de segurança actual (SAFE/ENGAGED/CHARGING/NOT_SAFE)

    // NOT_SAFE tem prioridade sobre os outros estados
    if (!bms_ok || !imd_ok || !esdb_flag || !bms_state_active)
    {
        ns = BMS_RLY_NOT_SAFE; /* inclui SLEEP/SHUTDOWN/UNINIT/FAULT/RING_RECOVERY */
    }

    else if (charger_opto)
    {
        ns = BMS_RLY_CHARGING; /* carregador ligado -> nunca SAFE/ENGAGED */
    }
    else if (!tsms_flag)
    {
        ns = BMS_RLY_SAFE;
    }
    else
    {
        ns = BMS_RLY_ENGAGED;
    }

    // Atualiza o estado se houver mudança
    if (ns != s_state)
    {
        s_state = ns;
        s_state_change = true;
    }

    /* --- 9) Acções de entrada de estado --- */
    if (s_state_change)
    {
        switch (s_state)
        {
            case BMS_RLY_NOT_SAFE:
            case BMS_RLY_SAFE:
                /* Pré-carga desligada; relé de DESCARGA/BLEED (PC2) ligado se
                 * o bus ainda tem tensão (>5 V) para o descarregar (transitório;
                 * o passo 7 abre-o ao chegar a ≤5 V).
                 * ⚠ PC2 é um relé de bleed (descarga do bus): fechá-lo aqui DESCARREGA o bus (correcto numa
                 *   emergência), não liga potência ao inversor. */
                RLY_OPEN(BMS_PRE_CHARGE_PORT, BMS_PRE_CHARGE_PIN);
                s_pre_charge_enable = false;
                if (!s_dis_charge_enable && bus_ge_min)
                {
                    RLY_CLOSE(BMS_DISCHARGE_PORT, BMS_DISCHARGE_PIN);
                    s_dis_charge_enable = true;
                }
                break;

            case BMS_RLY_CHARGING:
                /* Na entrada em CHARGING, desliga a pre-carga e comanda o
                 * bleed conforme BMS_BLEED_HOLD_IN_CHARGING. A posicao fisica
                 * dos contactos e a descarga efetiva nao sao verificadas aqui. */
                RLY_OPEN(BMS_PRE_CHARGE_PORT, BMS_PRE_CHARGE_PIN);
                s_pre_charge_enable = false;
#if (BMS_BLEED_HOLD_IN_CHARGING)
                /* HOLD: comanda o bleed fechado; o passo 7 preserva esse
                 * comando enquanto o estado continuar a ser CHARGING. */
                RLY_CLOSE(BMS_DISCHARGE_PORT, BMS_DISCHARGE_PIN);
                s_dis_charge_enable = true;
#else
                /* Fecha se bus >= 5 V; o passo 7 abre quando bus <= 5 V.
                 * Esta opcao nao limita a dissipacao se o bus continuar alimentado. */
                if (!s_dis_charge_enable && bus_ge_min)
                {
                    RLY_CLOSE(BMS_DISCHARGE_PORT, BMS_DISCHARGE_PIN);
                    s_dis_charge_enable = true;
                }
#endif
                break;

            case BMS_RLY_ENGAGED:
                /* Bleed desligado (não se sangra o bus em condução); pré-carga
                 * adiada BMS_PRECHARGE_DELAY_MS. A pré-carga sobe B+; quando B+
                 * atinge o limiar programado, o INVERSOR (Sevcon Gen4) fecha o
                 * Line_contactor autonomamente — o MCU não o comanda, só remove
                 * a pré-carga (passo 7) quando B+ chega a ~90% do pack. */
                RLY_OPEN(BMS_DISCHARGE_PORT, BMS_DISCHARGE_PIN);
                s_dis_charge_enable = false;
                s_engaged_tick = now_ms;
                break;

            default:
                break;
        }
        s_state_change = false;
    }

    /* --- 10) Pré-carga adiada do estado ENGAGED (não-bloqueante) --- */
    if ((s_state == BMS_RLY_ENGAGED) && (!s_pre_charge_enable) &&
        ((now_ms - s_engaged_tick) >= BMS_PRECHARGE_DELAY_MS) && bus_lt_90)
    {
        RLY_CLOSE(BMS_PRE_CHARGE_PORT, BMS_PRE_CHARGE_PIN);
        s_pre_charge_enable = true;
    }

    /* --- 11) LED cluster (conduzido pelo estado) --- */
    bool red = false, green = false, blue = false;
    switch (s_state)
    {
        case BMS_RLY_NOT_SAFE:
            red = true;
            break;
        case BMS_RLY_SAFE:
            green = ((now_ms % 1000U) < 500U);
            break; /* 1 Hz */
        case BMS_RLY_ENGAGED:
            green = true;
            break;
        case BMS_RLY_CHARGING:
            blue = true;
            break;
        default:
            break;
    }
    HAL_GPIO_WritePin(BMS_LED_RED_PORT, BMS_LED_RED_PIN, red ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(BMS_LED_GREEN_PORT, BMS_LED_GREEN_PIN, green ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(BMS_LED_BLUE_PORT, BMS_LED_BLUE_PIN, blue ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/* =========================================================================
 * ACESSORES
 * ========================================================================= */

/**
 * @brief  Devolve o estado de segurança actual (SAFE/ENGAGED/CHARGING/NOT_SAFE)
 */
BMS_RelayState_t BMS_Relays_GetState(void)
{
    return s_state;
}

/**
 * @brief  Devolve o nome legível do estado de segurança (para telemetria/debug)
 */
const char *BMS_Relays_GetStateString(void)
{
    switch (s_state)
    {
        case BMS_RLY_SAFE:
            return "SAFE";
        case BMS_RLY_ENGAGED:
            return "ENGAGED";
        case BMS_RLY_CHARGING:
            return "CHARGING";
        case BMS_RLY_NOT_SAFE:
            return "NOT_SAFE";
        default:
            return "UNKNOWN";
    }
}

/**
 * @brief Copia os monitores filtrados e o estado logico das saidas para out.
 *
 * As entradas sao as ultimas amostras aceites pelo debounce. As saidas sao
 * flags internas de comando, nao realimentacao de contactos. Em bancada o
 * modulo nao e amostrado pela tarefa; este snapshot nao prova estado fisico.
 * Se out for NULL, retorna sem escrever.
 */
void BMS_Relays_GetMonitors(BMS_RelayMonitors_t *out)
{
    if (out == NULL)
    {
        return;
    }

    out->imd_ok = (s_mon[MON_IMD].stable != 0U);
    out->tsms = (s_mon[MON_TSMS].stable != 0U);
    out->esdb = (s_mon[MON_ESDB].stable != 0U);
    out->esdb_charger = (s_mon[MON_ESDB_CHG].stable != 0U);
    out->charger = (s_mon[MON_CHARGER].stable != 0U);

    out->pre_charge_closed = s_pre_charge_enable;
    out->discharge_closed = s_dis_charge_enable;
    out->charge_relay_closed = s_charger_relay_enable;
    out->bms_relay_closed = !s_bms_relay_open_latched;
}
