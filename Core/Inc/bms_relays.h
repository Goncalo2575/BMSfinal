/**
 * @file    bms_relays.h
 * @brief   BMS - Máquina de estados de segurança + actuação de relés + LED cluster
 *
 *  ── O MCU COMANDA 5 RELÉS ────────────────────────────────────────────────
 *    PC0 pre_charge_relay · PC2 discharge_relay (bleed) · PC1 charge_relay ·
 *    PA6 BMS_charge_relay · PC4 BMS_relay   (BMS_relay e BMS_charge_relay têm
 *    a MESMA lógica: abrem em falha do BMS e ficam em latch).
 *
 *  ── ESTADOS ─────────────────────────────────────────
 *    SAFE      : verde intermitente 1 Hz   (TSMS aberto; pre-carga desactivada)
 *    ENGAGED   : verde contínuo            (TSMS fechado -> pré-carga/condução)
 *    CHARGING  : azul contínuo             (carregador ligado)
 *    NOT_SAFE  : vermelho contínuo         (BMS/IMD/ESDB aberto) [prioridade]
 *  Em bancada, Init/Task nao actuam nos GPIO. Estes estados descrevem
 *  a logica operacional legada, cuja integracao fisica requer validacao.
 *
 *  ── MAPA DE PINOS (CONFIRMAR polaridade real contra o esquema) ──────────
 *  SAÍDAS (relés, active-high: SET = relé fechado/energizado):
 *    PC0  pré-carga        PC1  charge (relé do carregador)
 *    PC2  descarga    PC4  BMS_relay        PA6  BMS_charge
 *  SAÍDAS (LED):
 *    PA15 verde   PC11 vermelho   PC12 azul
 *    
 *  ENTRADAS (monitor, pull-down fail-safe, active-high: SET = activo/fechado):
 *    PB0  IMD status   PC8  TSMS   PC6  ESDB   PB14 ESDB charger   PB12 charger signal                 
 *
 */

#ifndef BMS_RELAYS_H
#define BMS_RELAYS_H

#include "bq796xx_bms.h"

/* =========================================================================
 * CONFIGURAÇÃO DE PINOS  
 * ========================================================================= */
/* --- Saídas: relés --- */
#define BMS_PRE_CHARGE_PORT   GPIOC
#define BMS_PRE_CHARGE_PIN    GPIO_PIN_0
#define BMS_CHARGE_RELAY_PORT GPIOC
#define BMS_CHARGE_RELAY_PIN  GPIO_PIN_1
#define BMS_DISCHARGE_PORT    GPIOC
#define BMS_DISCHARGE_PIN     GPIO_PIN_2
#define BMS_RELAY_PORT        GPIOC
#define BMS_RELAY_PIN         GPIO_PIN_4
#define BMS_BMSCHARGE_PORT    GPIOA
#define BMS_BMSCHARGE_PIN     GPIO_PIN_6

/* --- Saídas: LED cluster --- */
#define BMS_LED_GREEN_PORT GPIOA
#define BMS_LED_GREEN_PIN  GPIO_PIN_15
#define BMS_LED_RED_PORT   GPIOC
#define BMS_LED_RED_PIN    GPIO_PIN_11
#define BMS_LED_BLUE_PORT  GPIOC
#define BMS_LED_BLUE_PIN   GPIO_PIN_12

/* --- Entradas: monitorização --- */
#define BMS_IMD_STATUS_PORT  GPIOB
#define BMS_IMD_STATUS_PIN   GPIO_PIN_0
#define BMS_IMD_Measure_PORT GPIOB
#define BMS_IMD_Measure_PIN  GPIO_PIN_1
#define BMS_TSMS_PORT        GPIOC
#define BMS_TSMS_PIN         GPIO_PIN_8
#define BMS_ESDB_PORT        GPIOC
#define BMS_ESDB_PIN         GPIO_PIN_6
#define BMS_ESDB_CHG_PORT    GPIOB
#define BMS_ESDB_CHG_PIN     GPIO_PIN_14
#define BMS_CHARGER_SIG_PORT GPIOB
#define BMS_CHARGER_SIG_PIN  GPIO_PIN_12

/* =========================================================================
 * PARÂMETROS  
 * ========================================================================= */
#define BMS_RELAY_DEBOUNCE_MS  100U  /* debounce dos monitores */
#define BMS_PRECHARGE_DELAY_MS 750U  /* atraso ENGAGED->pré-carga */
#define BMS_BUS_MIN_MV         5000U /* limiar "bus com tensão" (5 V) */
#define BMS_PRECHARGE_PCT_NUM  9U    /* 90%  da tensão do pack */
#define BMS_PRECHARGE_PCT_DEN  10U

/* Reclose automático do BMS_relay após a falha desaparecer.
 * 0 = latch aberto (igual ao ano passado: requer reset). 1 = re-fecha. */
#define BMS_RELAY_AUTO_RECLOSE 0

/* Comportamento do rele de descarga/bleed (PC2) durante CHARGING.
 * 1 = fecha na entrada do estado e impede a auto-abertura enquanto CHARGING.
 * 0 = fecha na entrada se bus >= 5 V; a tarefa abre-o quando bus <= 5 V.
 * A decisao usa a tensao do bus do inversor: nao mede a corrente de descarga
 * nem garante, por si so, que o bus esteja a 0 V. A integracao deve confirmar
 * a localizacao do resistor, a sua dissipacao e o isolamento face ao pack.
 * O valor predefinido pressupoe o bleed no lado do inversor.
 */
#define BMS_BLEED_HOLD_IN_CHARGING 1

/* =========================================================================
 * ESTRUTURAS
 * ========================================================================= */
typedef enum
{
    BMS_RLY_SAFE = 0,
    BMS_RLY_ENGAGED,
    BMS_RLY_CHARGING,
    BMS_RLY_NOT_SAFE
} BMS_RelayState_t;

typedef struct
{
    /* Entradas  */
    bool imd_ok;       /* PB0  */
    bool tsms;         /* PC8  */
    bool esdb;         /* PC6  */
    bool esdb_charger; /* PB14 */
    bool charger;      /* PB12 */

    /* Saídas  */
    bool pre_charge_closed;   /* PC0  */
    bool discharge_closed;    /* PC2  */
    bool charge_relay_closed; /* PC1  */
    bool bms_relay_closed;    /* PC4/PA6  */
} BMS_RelayMonitors_t;

/* =========================================================================
 * FUNCOES
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
void BMS_Relays_Init(void);

/**
 * @brief Executa a maquina de estados de reles e LEDs fora da bancada.
 *
 * Em BMS_BENCH_MODE retorna sem actuacao. Na integracao operacional, chamar
 * frequentemente com o tick em ms; as temporizacoes sao nao bloqueantes.
 * A autorizacao local legada depende de faults/pending e nao substitui
 * a revisao de interlocks, sensores e polaridades fisicas.
 */
void BMS_Relays_Task(BMS_Handle_t *hbms, uint32_t now_ms);

/** @brief  Estado de segurança actual. */
BMS_RelayState_t BMS_Relays_GetState(void);

/** @brief  Nome legível do estado (para telemetria/debug). */
const char *BMS_Relays_GetStateString(void);

/**
 * @brief Copia os monitores filtrados e o estado logico das saidas para out.
 *
 * As entradas sao as ultimas amostras aceites pelo debounce. As saidas sao
 * flags internas de comando, nao realimentacao de contactos. Em bancada o
 * modulo nao e amostrado pela tarefa; este snapshot nao prova estado fisico.
 * Se out for NULL, retorna sem escrever.
 */
void BMS_Relays_GetMonitors(BMS_RelayMonitors_t *out);

#endif /* BMS_RELAYS_H */
