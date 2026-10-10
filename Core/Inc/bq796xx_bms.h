/**
 * @file bq796xx_bms.h
 * @brief STM32F446RE + BQ79600-Q1 + dois BQ79616-Q1, 15S por slave.
 *
 * Revisao 3.4.4-BENCH: bancada com diagnostico FIRST/LAST, captura de transporte
 * e classificacao detalhada de falhas da bridge/slaves.
 * Esta camada publica decisoes LOGICAS; nao acciona os reles.
 * As chamadas comentadas da aplicacao continuam comentadas.
 * Nao constitui uma certificacao de seguranca funcional.
 *
 * GUIA DE LEITURA:
 * - main_bms_app.c organiza arranque, tarefa periodica e recuperacao.
 * - bq796xx_bms.c monta frames UART e configura e endereca os dispositivos.
 * - bq796xx_bms_monitor.c converte medicoes, interpreta falhas e inibe permissoes.
 * - bms_master_comm.c mostra o estado ja adquirido pela USART2.
 * - bms_relays.c contem a actuacao legada, bloqueada em modo de bancada.
 *
 * VOCABULARIO:
 * - Handle: estrutura persistente que guarda perifericos, medicoes e estado.
 * - ISR/callback: rotina chamada por interrupcao; nao pode bloquear na UART BQ.
 * - DMA: periferico copia os bytes recebidos para rx_buf sem copia pela CPU.
 * - Readback: leitura que confirma se uma escrita de configuracao foi aceite.
 * - Latch: indicador retido ate reconhecimento/reset explicito.
 * - Flags/mascara: cada bit representa uma condicao; varias podem coexistir.
 * - Raw: amostra antes do IIR de software; o LPF interno do BQ continua activo.
 * - Frescura: uma medicao so e utilizavel se for valida e suficientemente recente.
 *
 * Unidades: sufixos _mv, _ms e _us significam milivolts, milissegundos e
 * microssegundos. Arrays de celulas usam indice zero para C1; o endereco
 * do slave no cabo e diferente do seu indice fisico no array.
 */
#ifndef BQ796XX_BMS_H
#define BQ796XX_BMS_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>
#include "stm32f4xx_hal.h"

/* =========================================================================
 * 01 - IDENTIFICACAO DA ENTREGA
 * =========================================================================
 * Versao para rastrear ficheiros/logs. Nao misturar fontes de revisoes diferentes.
 */
#define BMS_FW_VERSION_MAJOR 3U
#define BMS_FW_VERSION_MINOR 4U
#define BMS_FW_VERSION_PATCH 4U

/* Bancada: aquisicao/diagnostico, SEM permissao BMS_OK ou balanceamento.
 * Alterar este define NAO reactiva nenhuma chamada comentada.
 * A integracao operacional exige ainda validar a cadeia fisica de corte,
 * sensores, diagnosticos e as chamadas de aquisicao/actuacao (README). */
#ifndef BMS_BENCH_MODE
#define BMS_BENCH_MODE 1U
#endif
#ifndef BMS_OPERATIONAL_INTEGRATION_CONFIRMED
#define BMS_OPERATIONAL_INTEGRATION_CONFIRMED 0U
#endif
#if (BMS_BENCH_MODE != 0U) && (BMS_BENCH_MODE != 1U)
#error "BMS_BENCH_MODE deve valer 0 ou 1"
#endif
#if (BMS_BENCH_MODE == 0U) && (BMS_OPERATIONAL_INTEGRATION_CONFIRMED != 1U)
#error "Validar integracao e README antes de compilar em modo operacional"
#endif

/* =========================================================================
 * 02 - TOPOLOGIA, PROTOCOLO E CAPACIDADE DOS BUFFERS
 * =========================================================================
 * Dois BQ79616, 15 celulas cada, ligados atraves do BQ79600.
 * Single Write maximo: INIT(1)+DEV(1)+REG(2)+DATA(8)+CRC(2)=14 bytes.
 * Single Read responde com DATA(N)+cabecalho/CRC(6). Stack Read recebe
 * um frame por slave: para N=128 sao 2*(128+6)=268 bytes.
 * Estas quantidades nao sao tensoes nem enderecos de registo.
 */
#define BMS_NUM_SLAVES        2U
#define BMS_CELLS_PER_SLAVE   15U
#define BMS_TOTAL_CELLS       (BMS_NUM_SLAVES * BMS_CELLS_PER_SLAVE)
#define BMS_UART_BAUDRATE     1000000UL
#define BMS_UART_TIMEOUT_MS   50U
#define BMS_MAX_WRITE_BYTES   8U
#define BMS_MAX_READ_BYTES    128U
#define BMS_RESPONSE_OVERHEAD 6U
#define BMS_MAX_FRAME_SIZE    14U
#define BMS_RX_BUFFER_SIZE    (BMS_NUM_SLAVES * (BMS_MAX_READ_BYTES + BMS_RESPONSE_OVERHEAD))
/* Alias historico: capacidade TOTAL, incluindo cabecalhos e CRC. */
#define BMS_MAX_RESPONSE_PAYLOAD BMS_RX_BUFFER_SIZE
#define BMS_DEVICE_ADDR_MAX      0x3FU
/* =========================================================================
 * 03 - CADENCIA E VALIDADE DOS DADOS
 * =========================================================================
 * Todos estes tempos sao em ms. A flag valid nao substitui a verificacao
 * de idade: uma resposta antiga nao prova que a ligacao ainda funciona.
 * As margens sao configuracao desta aplicacao, nao tolerancias garantidas TI.
 */
#define BMS_TASK_PERIOD_MS 100U
/* Orcamento inicial de engenharia; validar contra os requisitos do veiculo. */
#define BMS_MEASUREMENT_MAX_AGE_MS 300U
#define BMS_HW_STATUS_MAX_AGE_MS   300U
#define BMS_CONFIG_RECHECK_MS      1000U
#define BMS_RECOVERY_PERIOD_MS     5000U
#define BMS_RECOVERY_MAX_ATTEMPTS  3U
#define BMS_NFAULT_DIAG_PERIOD_MS  100U
#define BMS_DEBUG_REPEAT_MS        1000U /* Repetir diagnostico no maximo a 1 Hz se nada mudou. */
/* Em bancada, conservar contexto em vez de resetar por IWDG. */
#define BMS_BENCH_HOLD_ON_INIT_FAILURE 1U

/* O IWDG e iniciado pelo CubeMX. 250 e reload, logo (250+1)*64/f_LSI.
 * Aproximadamente 502 ms com LSI de 32 kHz; considerar a tolerancia do LSI. */
#define BMS_IWDG_PRESCALER       IWDG_PRESCALER_64
#define BMS_IWDG_RELOAD          250U
#define BMS_VOLTAGE_FILTER_ALPHA 4U
#define BMS_VOLTAGE_FILTER_SCALE 65536UL /* Estado Q16.16, publicacao em mV. */
#define BMS_SOC_TABLE_SIZE       21U
#define BMS_NTC_TABLE_SIZE       21U
#define BMS_COMM_CLEAR_BITS      18U
/* =========================================================================
 * 04 - ENDERECOS DOS DISPOSITIVOS, NAO DOS REGISTOS
 * =========================================================================
 * DIR0: bridge=0, S1=1, S2=2. DIR1: bridge=0, S2=1, S1=2.
 * BMS_GetSlaveAddress escolhe o identificador no sentido actual.
 * 0x02 tambem pode ser uma mascara de bit noutro contexto, sem conflito.
 */
#define BMS_ADDR_BRIDGE 0x00U
#define BMS_ADDR_SLAVE1 0x01U
#define BMS_ADDR_SLAVE2 0x02U

/* =========================================================================
 * 05 - PRIMEIRO BYTE DOS COMANDOS: INIT
 * =========================================================================
 * Bit 7=1 para comando; bits6:4=tipo; bit3 reservado; bits2:0=N-1
 * para escritas de 1..8 bytes. Em leituras, INIT e fixo e N-1 vai no DATA.
 * Com bridge BQ79600, usar Single Read ou Stack Read, nao Broadcast Read.
 * Fontes: TI-600 pp.15-16. Nao confundir INIT com endereco de dispositivo.
 */
#define INIT_BASE_SINGLE_READ     0x80U
#define INIT_BASE_SINGLE_WRITE    0x90U
#define INIT_BASE_STACK_READ      0xA0U
#define INIT_BASE_STACK_WRITE     0xB0U
#define INIT_BASE_BROADCAST_READ  0xC0U /* Nao utilizado com BQ79600. */
#define INIT_BASE_BROADCAST_WRITE 0xD0U
#define INIT_BASE_BCAST_REV_WRITE 0xE0U
/* Apenas apos validar 1 <= N <= 8; as funcoes publicas fazem essa validacao. */
#define BMS_INIT(base, N) ((uint8_t)((base) | ((uint8_t)(N) - 1U)))

/* Enderecamento/controlo (registos comuns, excepto COMM_CTRL). */
/* =========================================================================
 * 06 - REGISTOS DE ENDERECAMENTO E CONTROLO DA REDE
 * =========================================================================
 * REG_* indica ONDE ler/escrever. CTRL1_* e COMM_CTRL_* abaixo indica O QUE
 * escrever. DIR0/DIR1/CONTROL1 existem na bridge e nos slaves. COMM_CTRL
 * 0x0308 pertence aos slaves: define STACK_DEV e TOP_STACK.
 */
#define REG_BRIDGE_DIR0_ADDR 0x0306U
#define REG_BRIDGE_DIR1_ADDR 0x0307U
#define REG_STACK_COMM_CTRL  0x0308U
#define REG_BRIDGE_COMM_CTRL REG_STACK_COMM_CTRL /* Alias: so existe nos slaves. */
#define REG_BRIDGE_CONTROL1  0x0309U
#define REG_CONTROL2         0x030AU
/* =========================================================================
 * 07 - REGISTOS EXCLUSIVOS DA BRIDGE BQ79600
 * =========================================================================
 * DEV_CONF1=0x2001 tem valor inicial 0x14 nesta configuracao.
 * Resumo de falhas em 0x2100; detalhes ate 0x2105; reset proprio 0x2030.
 * Os registos de reset 0x0331/0x0332 dos slaves nao pertencem a bridge.
 */
#define REG_BRIDGE_DEV_CONF1      0x2001U
#define BRIDGE_DEV_CONF1_DEFAULT  0x14U
#define REG_BRIDGE_FAULT_MSK      0x2020U
#define BRIDGE_FAULT_MSK_REQUIRED 0x00U
#define REG_BRIDGE_FAULT_RST      0x2030U
#define REG_BRIDGE_FAULT_SUMMARY  0x2100U
#define REG_BRIDGE_FAULT_REG      0x2101U
#define REG_BRIDGE_FAULT_SYS      0x2102U
#define REG_BRIDGE_FAULT_PWR      0x2103U
#define REG_BRIDGE_FAULT_COMM1    0x2104U
#define REG_BRIDGE_FAULT_COMM2    0x2105U
#define BRIDGE_SUMMARY_PWR        0x01U
#define BRIDGE_SUMMARY_SYS        0x02U
#define BRIDGE_SUMMARY_REG        0x04U
#define BRIDGE_SUMMARY_COMM       0x08U
#define BRIDGE_SYS_CTS            0x04U
#define BRIDGE_SYS_CTL            0x08U
#define BRIDGE_SYS_COMM_TIMEOUTS  (BRIDGE_SYS_CTS | BRIDGE_SYS_CTL)
#define BRIDGE_COMM1_FCOMM_DET    0x40U
#define BRIDGE_COMM1_HB_FAIL      0x10U
#define BRIDGE_COMM1_HB_FAST      0x08U
#define BRIDGE_COMM1_HB_ERRORS    (BRIDGE_COMM1_HB_FAIL | BRIDGE_COMM1_HB_FAST)
#define BRIDGE_COMM1_COMMCLR_DET  0x02U
#define BRIDGE_COMM1_STOP_DET     0x01U
#define BRIDGE_COMM2_LINK_ERR     0x0FU
/* =========================================================================
 * 08 - SINCRONIZACAO DLL DOS SLAVES
 * =========================================================================
 * 0x0343..0x034A sao os oito registos usados pela sequencia TI de
 * sincronizacao: oito comandos separados de escrita e depois de leitura.
 * Nao sao os resultados ADC nem a area a programar em OTP.
 */
#define REG_ECC_DATA1 0x0343U
#define REG_ECC_DATA8 0x034AU

/* CRC da configuracao VOLATIL: nao se grava/programa OTP neste driver.
 * ATENCAO A ORDEM: BMS_CalculateCRC16 devolve a palavra reflectida. Para a
 * imagem factory default devolve 0xF331; por isso o byte numerico LOW=0x31
 * vai para CUST_CRC_HI e o byte numerico HIGH=0xF3 vai para CUST_CRC_LO,
 * exactamente como a tabela TI (31 F3). Nao trocar estes dois bytes.
 * Cobertura: 0x0000..0x0035, incluindo SPARE/RSVD lidos, nunca reescritos. */
/* =========================================================================
 * 09 - CRC INTERNO DA CONFIGURACAO DOS SLAVES
 * =========================================================================
 * CRC dos 54 bytes 0x0000..0x0035, distinto do CRC dos frames UART.
 * Ao mudar ACTIVE_CELL/OV/UV/filtro, o host tem de actualizar a referencia.
 * Escrita em CUST_CRC_HI/LO e VOLATIL; nao ha programacao de OTP.
 * Referencia: TI-616 sec.7.3.6.4.4.1, p.84 e mapa pp.113-115.
 */
#define REG_CUST_SHADOW_START    0x0000U
#define BMS_CUST_SHADOW_BYTES    54U
#define REG_CUST_CRC_HI          0x0036U
#define REG_CUST_CRC_RSLT_HI     0x050CU
#define REG_SLAVE_FAULT_OTP      0x0535U
#define REG_SLAVE_FAULT_SYS      0x0536U
#define DEV_STAT_CUST_CRC_DONE   0x20U
#define SLAVE_FAULT_OTP_CUST_CRC 0x10U
#define SLAVE_COMM3_FCOMM_DET    0x08U
#define SLAVE_FAULT_COMM3_FCOMM  SLAVE_COMM3_FCOMM_DET /* Alias historico. */
#define SLAVE_RST2_OTP_CRC       0x40U
#define SLAVE_RST2_FCOMM         0x10U
#define BRIDGE_FAULT_COMM1_FCOMM BRIDGE_COMM1_FCOMM_DET /* Alias historico. */
#define BRIDGE_RST_FCOMM         0x20U
#define BMS_CUST_CRC_TIMEOUT_MS  50U
/* =========================================================================
 * 10 - CONFIGURACAO E ESTADO DO ADC/COMPARADORES
 * =========================================================================
 * ACTIVE_CELL codifica numero de canais; OV/UV definem limiares.
 * ADC_CTRL1 inicia/paralisa o ADC; OVUV_CTRL inicia comparadores independentes.
 * ADC_STAT1/DEV_STAT sao resultados de estado, nao comandos de arranque.
 */
#define REG_ACTIVE_CELL      0x0003U
#define REG_ADC_CONF1        0x0007U
#define REG_OV_THRESH        0x0009U
#define REG_UV_THRESH        0x000AU
#define REG_GPIO_CONF1       0x000EU
#define REG_GPIO_CONF2       0x000FU
#define REG_ADC_CTRL1        0x030DU
#define REG_OVUV_CTRL        0x032CU
#define REG_DIAG_STAT        0x0526U
#define REG_ADC_STAT1        0x0527U
#define REG_BAL_STAT         0x052BU
#define REG_DEV_STAT         0x052CU
#define ADC_STAT1_MAIN_READY 0x01U
#define DEV_STAT_MAIN_RUN    0x01U
#define DEV_STAT_OVUV_RUN    0x08U
#define CONTROL2_TSREF_EN    0x01U
/* GPIO1-4 como ADC apenas. OTUT/OTCB nao sao declarados activos.
 * Os limiares autonomos de temperatura dependem do NTC/circuito real. */
#define GPIO_CONF_ADC_ONLY 0x12U
#define ADC_CONF1_26HZ     0x02U

/* =========================================================================
 * 11 - RESULTADOS DAS TENSOES: ORDEM INVERSA DOS CANAIS
 * =========================================================================
 * O endereco mais baixo corresponde a celula mais alta: VCELL16=0x0568.
 * Para 15S le-se desde VCELL15=0x056A ate VCELL1_LO=0x0587, 30 bytes.
 * O driver reorganiza para o array [0]=C1 ... [14]=C15.
 * Pressuposto ja existente: os 15 canais activos sao os inferiores.
 */
#define REG_VCELL16_HI       0x0568U
#define REG_VCELL_STRIDE     2U
#define REG_VCELL15_HI       0x056AU
#define REG_VCELL1_HI        0x0586U
#define BMS_VCELL_READ_BYTES (BMS_CELLS_PER_SLAVE * REG_VCELL_STRIDE)
/* =========================================================================
 * 12 - REGISTOS AUXILIARES MANTIDOS, FORA DO FOCO DESTA REVISAO
 * =========================================================================
 * BUSBAR, TSREF e GPIO nao sao tensoes de celula.
 * As rotinas NTC/HV e as chamadas desactivadas foram conservadas.
 * Os valores abaixo nao sao revalidados contra um novo esquema nesta entrega.
 */
#define REG_BUSBAR_HI        0x0588U
#define REG_BUSBAR_LO        0x0589U
#define REG_TSREF_HI         0x058CU
#define REG_TSREF_LO         0x058DU
#define REG_GPIO1_HI         0x058EU
#define REG_GPIO1_LO         0x058FU
#define REG_GPIO2_HI         0x0590U
#define REG_GPIO2_LO         0x0591U
#define REG_GPIO3_HI         0x0592U
#define REG_GPIO3_LO         0x0593U
#define REG_GPIO4_HI         0x0594U
#define REG_GPIO4_LO         0x0595U
#define BMS_NUM_TEMP_SENSORS 3U
#define BMS_AUX_READ_BYTES   10U /* TSREF, GPIO1, GPIO2, GPIO3, GPIO4. */
#define BMS_ADC_INVALID_CODE 0x8000U
/* Escalas em 0.01 microvolt/LSB: TI-616, pagina 13. */
#define BMS_CELL_LSB_X100_UV      19073UL
#define BMS_GPIO_LSB_X100_UV      15259UL
#define BMS_TSREF_LSB_X100_UV     16954UL
#define BMS_INVALID_TEMPERATURE_C INT16_MIN
#define BMS_TSREF_MIN_MV          4500U
#define BMS_TSREF_MAX_MV          5500U
#define BMS_NTC_RATIO_MIN         100U
#define BMS_NTC_RATIO_MAX         9500U
/* Divisor e NTC mantidos do projecto; confirmar no esquema/calibracao. */
#define HV_BUS_ATTENUATION_NUM  2711U
#define HV_BUS_ATTENUATION_DEN  100U
#define PRECHARGE_PERCENT       90U
#define BMS_HV_MAX_PLAUSIBLE_MV 140000UL
/* Alias informativo; a decisao usa 90% do PACK MEDIDO, nao esta constante. */
#define PRECHARGE_THRESHOLD_MV 113400UL

/* =========================================================================
 * 13 - PINOS DA COMUNICACAO, CONFORME O PROJECTO RECEBIDO
 * =========================================================================
 * UART4 TX=PA0, RX=PA1, AF8; NFAULT=PA8.
 * O driver altera apenas TX para GPIO durante pings e restaura a UART.
 * O projecto CubeMX tem de configurar clocks, RX DMA e interrupcoes.
 * Nao se altera aqui o pinout de hardware.
 */
#define BMS_BRIDGE_TX_PORT GPIOA
#define BMS_BRIDGE_TX_PIN  GPIO_PIN_0
#define BMS_BRIDGE_RX_PIN  GPIO_PIN_1
#define BMS_BRIDGE_UART_AF GPIO_AF8_UART4
#define BMS_NFAULT_PORT    GPIOA
#define BMS_NFAULT_PIN     GPIO_PIN_8

/* =========================================================================
 * 14 - FALHAS DOS BQ79616: RESUMO, DETALHE E RESET
 * =========================================================================
 * FAULT_SUMMARY e lido; FAULT_RST1/2 recebem mascaras para limpar latches.
 * OV1/UV1/VCOW1 contem C9..C16; o byte 2 contem C1..C8.
 * Um latch limpo nao prova que a causa desapareceu. O scan seguinte valida.
 * VCOW=0 sem executar diagnostico open-wire nao e prova de fios intactos.
 */
#define REG_SLAVE_FAULT_SUMMARY     0x052DU
#define REG_SLAVE_FAULT_MSK1        0x0016U
#define REG_SLAVE_FAULT_MSK2        0x0017U
#define SLAVE_FAULT_MSK2_USED_BITS  0x7FU /* Bit 7 spare: conservar. */
#define SLAVE_SUMMARY_PWR           0x01U
#define SLAVE_SUMMARY_SYS           0x02U
#define SLAVE_SUMMARY_OVUV          0x04U
#define SLAVE_SUMMARY_OTUT          0x08U
#define SLAVE_SUMMARY_COMM          0x10U
#define SLAVE_SUMMARY_OTP           0x20U
#define SLAVE_SUMMARY_COMP          0x40U
#define SLAVE_SUMMARY_PROT          0x80U
#define SLAVE_SYS_CTS               0x04U
#define SLAVE_SYS_CTL               0x08U
#define SLAVE_SYS_COMM_TIMEOUTS     (SLAVE_SYS_CTS | SLAVE_SYS_CTL)
#define REG_SLAVE_FAULT_COMM1       0x0530U
#define REG_SLAVE_FAULT_COMM2       0x0531U
#define REG_SLAVE_FAULT_COMM3       0x0532U
#define SLAVE_COMM2_LINK_ERR        0xFFU /* Todos os bits pertencem a COML/COMH. */
#define SLAVE_COMM3_HB_FAIL         0x02U
#define SLAVE_COMM3_HB_FAST         0x01U
#define SLAVE_COMM3_HB_ERRORS       (SLAVE_COMM3_HB_FAIL | SLAVE_COMM3_HB_FAST)
#define REG_SLAVE_FAULT_PROT1       0x053AU
#define REG_SLAVE_FAULT_PROT2       0x053BU
#define REG_SLAVE_FAULT_OV1         0x053CU /* Celulas 9-16. */
#define REG_SLAVE_FAULT_OV2         0x053DU /* Celulas 1-8. */
#define REG_SLAVE_FAULT_UV1         0x053EU /* Celulas 9-16. */
#define REG_SLAVE_FAULT_UV2         0x053FU /* Celulas 1-8. */
#define REG_SLAVE_FAULT_OT          0x0540U
#define REG_SLAVE_FAULT_UT          0x0541U
#define REG_SLAVE_FAULT_COMP_GPIO   0x0543U
#define REG_SLAVE_FAULT_COMP_VCCB1  0x0545U
#define REG_SLAVE_FAULT_COMP_VCCB2  0x0546U
#define REG_SLAVE_FAULT_COMP_VCOW1  0x0548U /* Celulas 9-16. */
#define REG_SLAVE_FAULT_COMP_VCOW2  0x0549U /* Celulas 1-8. */
#define REG_SLAVE_FAULT_COMP_CBOW1  0x054BU
#define REG_SLAVE_FAULT_COMP_CBOW2  0x054CU
#define REG_SLAVE_FAULT_COMP_CBFET1 0x054EU
#define REG_SLAVE_FAULT_COMP_CBFET2 0x054FU
#define REG_SLAVE_FAULT_COMP_MISC   0x0550U
#define REG_SLAVE_FAULT_PWR1        0x0552U
#define REG_SLAVE_FAULT_PWR2        0x0553U
#define REG_SLAVE_FAULT_PWR3        0x0554U
#define SLAVE_PWR1_AVDD_OSC         0x02U
#define SLAVE_PWR3_AVDDUV_DRST      0x01U
#define REG_SLAVE_FAULT_RST1        0x0331U
#define REG_SLAVE_FAULT_RST2        0x0332U
#define REG_FAULT_RST1              REG_SLAVE_FAULT_RST1 /* Alias historico. */
#define REG_FAULT_RST2              REG_SLAVE_FAULT_RST2 /* Alias historico. */
#define SLAVE_FAULT_RST1_ALL        0xFFU
#define SLAVE_FAULT_RST2_ALL        0x7FU
#define FAULT_RST1_ALL              SLAVE_FAULT_RST1_ALL /* Alias historico. */
#define FAULT_RST2_ALL              SLAVE_FAULT_RST2_ALL /* Alias historico. */
#define BRIDGE_FAULT_RST_ALL        0xFFU
#define FAULT_CLEAR_VAL             FAULT_RST1_ALL /* Alias; NAO usar em FAULT_RST2. */
#define SLAVE_RST1_PWR              0x01U
#define SLAVE_RST1_SYS              0x02U
#define SLAVE_RST2_COMM1            0x01U
#define SLAVE_RST2_COMM2            0x02U
#define SLAVE_RST2_HB               0x04U
#define SLAVE_RST2_FTONE            0x08U
#define SLAVE_FAULT_RST1_STARTUP    SLAVE_RST1_SYS
#define SLAVE_FAULT_RST2_STARTUP                                                                   \
    (SLAVE_RST2_COMM1 | SLAVE_RST2_COMM2 | SLAVE_RST2_HB | SLAVE_RST2_FTONE | SLAVE_RST2_FCOMM)
#define BRIDGE_RST_SYS      0x02U
#define BRIDGE_RST_HB       0x08U
#define BRIDGE_RST_FTONE    0x10U
#define BRIDGE_RST_UART_SPI 0x40U
#define BRIDGE_RST_COML_H   0x80U
#define BRIDGE_FAULT_RST_STARTUP                                                                   \
    (BRIDGE_RST_SYS | BRIDGE_RST_HB | BRIDGE_RST_FTONE | BRIDGE_RST_FCOMM | BRIDGE_RST_UART_SPI |  \
     BRIDGE_RST_COML_H)

/* Indices de bytes, NAO enderecos: grupos esparsos compactados para debug. */
typedef enum
{
    BMS_FD_OTP = 0,
    BMS_FD_SYS,
    BMS_FD_PWR1,
    BMS_FD_PWR2,
    BMS_FD_PWR3,
    BMS_FD_PROT1,
    BMS_FD_PROT2,
    BMS_FAULT_DETAIL_BYTES
} BMS_FaultDetailIndex_t;

typedef enum
{
    BMS_FC_GPIO = 0,
    BMS_FC_VCCB1,
    BMS_FC_VCCB2,
    BMS_FC_VCOW1,
    BMS_FC_VCOW2,
    BMS_FC_CBOW1,
    BMS_FC_CBOW2,
    BMS_FC_CBFET1,
    BMS_FC_CBFET2,
    BMS_FC_MISC,
    BMS_FAULT_COMP_BYTES
} BMS_FaultCompIndex_t;

/* =========================================================================
 * 15 - DADOS/MASCARAS A ESCREVER NOS REGISTOS
 * =========================================================================
 * CONTROL1: DIR_SEL e bit7=0x80; ADDR_WR bit0=0x01.
 * 0x02 em CONTROL1 seria SOFT_RESET, nao a direccao.
 * COMM_CTRL: 0x02=STACK_DEV; 0x03=STACK_DEV|TOP_STACK.
 * ADC continuo com LPF: 0x08|0x04|0x02=0x0E. OVUV normal:0x04|0x01=0x05.
 */
#define CTRL1_SEND_WAKE          0x20U
#define CTRL1_SEND_SHUTDOWN      0x40U
#define CTRL1_ADDR_WR            0x01U
#define CTRL1_DIR_SEL            0x80U
#define COMM_CTRL_STACK_DEV      0x02U
#define COMM_CTRL_TOP_STACK      0x03U /* Valor completo: STACK_DEV | TOP_STACK. */
#define ADC_CTRL1_CONTINUOUS_LPF 0x0EU /* MAIN_MODE=10, MAIN_GO, LPF_VCELL_EN. */
#define ADC_CTRL1_STOP           0x04U /* MAIN_MODE=00, MAIN_GO=1. */
#define OVUV_CTRL_ENABLE         0x05U
#define ACTIVE_CELL_15S          0x09U
/* =========================================================================
 * 16 - LIMIARES DE HARDWARE: CODIGOS, NAO MILIVOLTS
 * =========================================================================
 * OV: gamas validas 2700..3000, 3500..3800, 4175..4475 mV, passo25mV;
 * codigos iniciais 0x02,0x12,0x22. Exemplo 4200 ->0x22+(4200-4175)/25=0x23.
 * UV: codigo=(mV-1200)/50, valido1200..3100mV. 2500 ->26=0x1A.
 * CELL_OV_MV/CELL_UV_MV sao os valores de software definidos mais abaixo.
 * As verificacoes de compilacao impedem deixar um par incoerente.
 */
/* LIMITES TEMPORARIOS DE BANCADA. Nao usar num pack real.
 * O objectivo actual e provocar/verificar OV/UV com fontes de baixa tensao.
 * BMS_BENCH_TEST_THRESHOLDS=1 selecciona 2700/1500 mV.
 * BMS_BENCH_TEST_THRESHOLDS=0 selecciona o par operacional 4200/3000 mV. */
#ifndef BMS_BENCH_TEST_THRESHOLDS
#define BMS_BENCH_TEST_THRESHOLDS 1U
#endif
#if BMS_BENCH_TEST_THRESHOLDS
#define OV_THRESH_VAL 0x02U /* TESTE: 2700 mV. */
#define UV_THRESH_VAL 0x06U /* TESTE: 1500 mV. */
#else
#define OV_THRESH_VAL 0x23U /* OPERACIONAL: 4200 mV. */
#define UV_THRESH_VAL 0x24U /* OPERACIONAL: 3000 mV. */
#endif

/* =========================================================================
 * 17 - REGISTOS DE BALANCEAMENTO, SEM ACTIVACAO NOVA
 * =========================================================================
 * Um temporizador por celula em0x0318..0x0327, de C16 para C1.
 * Uma mascara do software NAO e escrita directamente nestes registos.
 * Parar: escrever timers=0 e emitir BAL_GO, depois confirmar BAL_STAT.
 * O bloqueio de bancada e as precondicoes anteriores mantem-se.
 */
#define REG_CB_CELL16_CTRL            0x0318U
#define REG_CB_CELL1_CTRL             0x0327U
#define REG_CB_CELL_CTRL(n)           ((uint16_t)(REG_CB_CELL1_CTRL - ((uint16_t)(n) - 1U)))
#define CB_CELL_NUM_REGS              16U
#define CB_TIMER_OFF                  0x00U
#define CB_TIMER_ACTIVE               0x01U /* 10 segundos de tempo de balanceamento/celula. */
#define REG_MB_TIMER_CTRL             0x0329U
#define REG_BAL_CTRL1                 0x032EU
#define REG_BAL_CTRL2                 0x032FU
#define REG_BAL_CTRL3                 0x0330U
#define BAL_CTRL1_DUTY_5S             0x00U
#define BAL_CTRL2_IDLE                0x00U          /* Nao e uma ordem de paragem. */
#define BAL_CTRL2_STOP                BAL_CTRL2_IDLE /* Alias historico; timers=0 + GO param. */
#define BAL_CTRL2_AUTO_GO             0x23U          /* AUTO_BAL | BAL_GO | FLTSTOP_EN. */
#define BAL_STAT_CB_RUN               0x08U
#define BAL_STAT_MB_RUN               0x10U
#define BAL_STAT_CB_PAUSED            0x20U
#define BAL_STAT_ABORTFLT             0x04U
#define BAL_STAT_INVALID_CONF         0x80U
#define BMS_BALANCE_RESTART_DELAY_MS  1000U
#define BMS_BALANCE_VERIFY_TIMEOUT_MS 20U
#define CELL_BALANCE_DELTA_MV         20U
#define CELL_BALANCE_HYSTERESIS_MV    10U
#define CELL_BALANCE_MIN_MV           3800U
#define CELL_BALANCE_STOP_MV          5U

/* =========================================================================
 * 18 - TEMPORIZACOES DA BRIDGE/SLAVES
 * =========================================================================
 * Sufixo_US=microssegundos; _MS=milissegundos.
 * WAKE da bridge:2500..3000us. SHUTDOWN da bridge:>=12500us.
 * DIR_SEL: aguardar pelo menos 100 us. AFE: esperar antes de arrancar ADC.
 * Margens para2slaves e filtro26Hz conservadas; medir na placa para validar.
 */
#define DELAY_WAKE_PULSE_US     2750U  /* Dentro da janela 2500-3000 us da bridge. */
#define DELAY_SHUTDOWN_PULSE_US 13000U /* Margem sobre o minimo 12500 us. */
/* Obsoleto: um pulso no RX da bridge NAO e um HW_RESET da stack. */
#define DELAY_HWRESET_PULSE_US     40000U
#define DELAY_SLEEP_CONTACTOR_MS   100U /* Nao usado como prova de abertura fisica. */
#define BMS_DAISY_CHAIN_LATENCY_US 2000U
#define DELAY_OSC_STAB_MS          4U
#define DELAY_WAKE_PROPAGATION_MS  25U
#define DELAY_DIR_SEL_SWITCH_US    100U
#define DELAY_AFE_SETTLE_MS        5U
#define DELAY_TSREF_SETTLE_MS      10U
#define DELAY_ADC_SETTLE_MS        50U /* Margem para LPF 26 Hz (38 ms average). */
#define BMS_ADC_READY_TIMEOUT_MS   50U
#define BMS_WRITE_GAP_US           20U
#define BMS_SHUTDOWN_STACK_WAIT_MS 45U

/* =========================================================================
 * 19 - LIMITES DE SOFTWARE E POLINOMIO CRC
 * =========================================================================
 * Valores por celula em mV; nao confundir com a tensao total do pack.
 * Nesta entrega estao INTENCIONALMENTE em 2700/1500 mV para bancada.
 * IIR nao substitui os valores usados para detectar OV/UV.
 */
#if BMS_BENCH_TEST_THRESHOLDS
#define CELL_OV_MV      2700U /* TESTE DE BANCADA */
#define CELL_UV_MV      1500U /* TESTE DE BANCADA */
#define CELL_WARN_UV_MV 1600U
#define CELL_WARN_OV_MV 2500U
#else
#define CELL_OV_MV      4200U
#define CELL_UV_MV      3000U
#define CELL_WARN_UV_MV 3100U
#define CELL_WARN_OV_MV 4150U
#endif
#define CELL_TEMP_MAX_C      70U
#define CELL_TEMP_WARN_C     60U
#define CELL_IMBALANCE_MV    50U
#define CRC16_INIT           0xFFFFU
#define CRC16_POLY_LSB_FIRST 0xA001U

/* Impedir que uma mudanca no .h deixe os comparadores e o software a
 * proteger limites diferentes. Os valores existentes NAO foram alterados.
 * Para limites de hardware intencionalmente diferentes, rever esta politica. */
#define BMS_OV_CODE_IS_VALID(c)                                                                    \
    ((((c) >= 0x02U) && ((c) <= 0x0EU)) || (((c) >= 0x12U) && ((c) <= 0x1EU)) ||                   \
     (((c) >= 0x22U) && ((c) <= 0x2EU)))
#define BMS_OV_CODE_TO_MV(c)                                                                       \
    (((c) <= 0x0EU)   ? (2700U + ((c) - 0x02U) * 25U)                                              \
     : ((c) <= 0x1EU) ? (3500U + ((c) - 0x12U) * 25U)                                              \
                      : (4175U + ((c) - 0x22U) * 25U))
#define BMS_UV_CODE_TO_MV(c) (1200U + (c) * 50U)
#if (BMS_NUM_SLAVES != 2U) || (BMS_CELLS_PER_SLAVE != 15U)
#error "Esta revisao foi configurada para dois BQ79616 com 15 canais cada"
#endif
#if !BMS_OV_CODE_IS_VALID(OV_THRESH_VAL) || (UV_THRESH_VAL > 0x26U)
#error "Codigo OV/UV nao suportado pelo BQ79616"
#endif
#if (BMS_OV_CODE_TO_MV(OV_THRESH_VAL) != CELL_OV_MV) ||                                            \
    (BMS_UV_CODE_TO_MV(UV_THRESH_VAL) != CELL_UV_MV)
#error "Limites de hardware e software OV/UV nao correspondem"
#endif
#if (BMS_BENCH_TEST_THRESHOLDS != 0U) && (BMS_BENCH_TEST_THRESHOLDS != 1U)
#error "BMS_BENCH_TEST_THRESHOLDS deve valer 0 ou 1"
#endif
#if (BMS_BENCH_TEST_THRESHOLDS != 0U) && (BMS_BENCH_MODE == 0U)
#error "Limites OV/UV de bancada activos: rever antes de usar pack real"
#endif
#if (CELL_UV_MV >= CELL_OV_MV) || (CELL_WARN_UV_MV <= CELL_UV_MV) ||                               \
    (CELL_WARN_OV_MV >= CELL_OV_MV) || (CELL_WARN_UV_MV >= CELL_WARN_OV_MV)
#error "Rever a ordem dos limites e avisos de tensao"
#endif
#if (BMS_VOLTAGE_FILTER_ALPHA == 0U) || (BMS_VOLTAGE_FILTER_ALPHA > BMS_VOLTAGE_FILTER_SCALE / 2U)
#error "Alpha do IIR deve estar entre 1 e 32768 para convergir a resolucao de 1 mV"
#endif

/* Compatibilidade com nomes antigos e valores validados nas tabelas TI. */
_Static_assert(REG_BRIDGE_COMM_CTRL == REG_STACK_COMM_CTRL, "Alias COMM_CTRL divergiu");
_Static_assert(REG_FAULT_RST1 == REG_SLAVE_FAULT_RST1, "Alias FAULT_RST1 divergiu");
_Static_assert(REG_FAULT_RST2 == REG_SLAVE_FAULT_RST2, "Alias FAULT_RST2 divergiu");
_Static_assert(FAULT_RST1_ALL == SLAVE_FAULT_RST1_ALL, "Alias RST1_ALL divergiu");
_Static_assert(FAULT_RST2_ALL == SLAVE_FAULT_RST2_ALL, "Alias RST2_ALL divergiu");
_Static_assert(FAULT_CLEAR_VAL == SLAVE_FAULT_RST1_ALL, "Alias FAULT_CLEAR_VAL divergiu");
_Static_assert(BAL_CTRL2_STOP == BAL_CTRL2_IDLE, "Alias BAL_CTRL2_STOP divergiu");
_Static_assert(BMS_MAX_RESPONSE_PAYLOAD == BMS_RX_BUFFER_SIZE, "Alias RX divergiu");
_Static_assert(SLAVE_FAULT_COMM3_FCOMM == SLAVE_COMM3_FCOMM_DET, "Alias slave FCOMM divergiu");
_Static_assert(BRIDGE_FAULT_COMM1_FCOMM == BRIDGE_COMM1_FCOMM_DET, "Alias bridge FCOMM divergiu");
_Static_assert(SLAVE_FAULT_RST1_STARTUP == 0x02U, "RST1 de arranque incorrecto");
_Static_assert(SLAVE_FAULT_RST2_STARTUP == 0x1FU, "RST2 de arranque incorrecto");
_Static_assert(BRIDGE_FAULT_RST_STARTUP == 0xFAU, "Reset bridge de arranque incorrecto");
_Static_assert(REG_SLAVE_FAULT_SYS == REG_SLAVE_FAULT_OTP + 1U, "OTP/SYS nao contiguos");
_Static_assert(REG_SLAVE_FAULT_PWR3 == REG_SLAVE_FAULT_PWR1 + 2U, "PWR nao contiguos");
_Static_assert(REG_SLAVE_FAULT_PROT2 == REG_SLAVE_FAULT_PROT1 + 1U, "PROT nao contiguos");
_Static_assert(BMS_FD_SYS == BMS_FD_OTP + 1 && BMS_FD_PWR3 == BMS_FD_PWR1 + 2 &&
                   BMS_FD_PROT2 == BMS_FD_PROT1 + 1,
               "Indices dos blocos de detalhe divergiram");
_Static_assert(BMS_FC_VCCB2 == BMS_FC_VCCB1 + 1 && BMS_FC_VCOW2 == BMS_FC_VCOW1 + 1 &&
                   BMS_FC_CBOW2 == BMS_FC_CBOW1 + 1 && BMS_FC_CBFET2 == BMS_FC_CBFET1 + 1,
               "Indices dos blocos COMP divergiram");

/* Valores existentes preservados para compatibilidade com a telemetria. */
/* =========================================================================
 * 20 - ESTADOS, RESULTADOS E FLAGS
 * =========================================================================
 * Estado indica a fase do driver. Status indica o resultado de uma chamada.
 * Fault flags podem conter varias causas em simultaneo. Warnings nao sao
 * permissoes. Os valores existentes foram mantidos para compatibilidade.
 */
typedef enum
{
    BMS_STATE_UNINITIALIZED = 0,
    BMS_STATE_INITIALIZING,
    BMS_STATE_IDLE,
    BMS_STATE_MONITORING,
    BMS_STATE_BALANCING,
    BMS_STATE_FAULT,
    BMS_STATE_SHUTDOWN,
    BMS_STATE_SLEEP,
    BMS_STATE_RING_RECOVERY
} BMS_State_t;

typedef enum
{
    BMS_CMD_SINGLE_WRITE = 0,
    BMS_CMD_SINGLE_READ,
    BMS_CMD_BROADCAST_WRITE,
    BMS_CMD_BROADCAST_READ,
    BMS_CMD_STACK_WRITE,
    BMS_CMD_STACK_READ,
    BMS_CMD_BROADCAST_WRITE_REV
} BMS_CmdType_t;

typedef enum
{
    BMS_FAULT_NONE = 0U,
    BMS_FAULT_OV = 0x1U,
    BMS_FAULT_UV = 0x2U,
    BMS_FAULT_OT = 0x4U,
    BMS_FAULT_COMM = 0x8U,
    BMS_FAULT_RING_BREAK = 0x10U,
    BMS_FAULT_OPEN_WIRE = 0x20U,
    BMS_FAULT_HB_FAIL = 0x40U,
    BMS_FAULT_CRC = 0x80U,
    BMS_FAULT_HARDWARE = 0x100U,
    BMS_FAULT_SENSOR = 0x200U,
    BMS_FAULT_STALE_DATA = 0x400U,
    BMS_FAULT_BALANCING = 0x800U,
    BMS_FAULT_UT = 0x1000U,
    BMS_FAULT_CONFIG = 0x2000U,
    BMS_FAULT_FRAME = 0x4000U,
    BMS_FAULT_PLATFORM = 0x8000U
} BMS_FaultFlag_t;

typedef enum
{
    BMS_OK = 0,
    BMS_ERR_TIMEOUT,
    BMS_ERR_CRC,
    BMS_ERR_COMM,
    BMS_ERR_INVALID_PARAM,
    BMS_ERR_FAULT_ACTIVE,
    BMS_ERR_INIT_FAILED,
    BMS_ERR_NOT_READY,
    BMS_ERR_FRAME,
    BMS_ERR_PLATFORM
} BMS_Status_t;

typedef enum
{
    BMS_WARN_NONE = 0U,
    BMS_WARN_OV = 1U,
    BMS_WARN_UV = 2U,
    BMS_WARN_TEMP = 4U,
    BMS_WARN_IMBALANCE = 8U,
    BMS_WARN_TEMP_UNAVAILABLE = 16U,
    BMS_WARN_HV_UNAVAILABLE = 32U,
    BMS_WARN_RING_DEGRADED = 64U
} BMS_WarningFlag_t;

/* Contexto compacto de erro para debug de bancada. FIRST nunca e substituido
 * depois do primeiro erro da sessao; LAST acompanha a causa mais recente.
 * expected/observed/mask valem 0 quando a operacao nao tem comparacao directa. */
typedef struct
{
    bool valid; /* false: os restantes campos nao descrevem uma ocorrencia. */
    BMS_Status_t status;
    const char *reason; /* Literal estatico: causa especifica da falha. */
    const char *step;
    uint32_t tick_ms;
    uint8_t direction;
    uint8_t device;
    uint16_t reg;
    uint32_t expected;
    uint32_t observed;
    uint32_t mask;
} BMS_ErrorContext_t;

/* =========================================================================
 * 21 - DADOS POR IDENTIDADE FISICA DO SLAVE
 * =========================================================================
 * Cada estrutura permanece associada a S1 ou S2; so o endereco usado no
 * frame muda com DIR0/DIR1. Valores e flags de validade sao campos distintos.
 * Os dois bytes expected_config_crc correspondem a ultima imagem verificada.
 */
typedef struct
{
    uint8_t address;                               /* DIR0, identidade fisica imutavel: 1 ou 2. */
    uint8_t address_rev;                           /* DIR1, identidade fisica imutavel: 2 ou 1. */
    uint16_t cell_voltage_mv[BMS_CELLS_PER_SLAVE]; /* Amostras antes do IIR de SW. */
    int16_t temperatures_c[BMS_NUM_TEMP_SENSORS];
    uint32_t fault_flags;
    uint8_t fault_summary_raw[4]; /* [0]=SUMMARY, [1..3]=COMM1..3 quando validos. */
    uint8_t fault_detail_raw[BMS_FAULT_DETAIL_BYTES]; /* OTP,SYS,PWR1..3,PROT1..2. */
    uint8_t fault_comp_raw[BMS_FAULT_COMP_BYTES]; /* GPIO,VCCB1/2,VCOW1/2,CBOW1/2,CBFET1/2,MISC. */
    uint8_t fault_ovuv_raw[4];                    /* OV1,OV2,UV1,UV2. */
    uint8_t fault_otut_raw[2];                    /* OT,UT. */
    uint8_t fault_detail_valid; /* Bits FAULT_SUMMARY dos grupos de detalhe lidos com sucesso. */
    uint16_t fault_comp_valid;  /* Bit i=byte fault_comp_raw[i] lido com sucesso. */
    bool fault_summary_valid, fault_comm_valid, fault_ovuv_valid, fault_otut_valid;
    uint8_t init_pwr_raw[3]; /* Snapshot ANTES do RST_PWR; conservado nos scans seguintes. */
    bool init_pwr_valid, init_pwr_cleared;
    bool ov_cell[BMS_CELLS_PER_SLAVE], uv_cell[BMS_CELLS_PER_SLAVE];
    bool comm_ok;
    uint16_t balance_mask;
    bool voltage_valid, temperature_valid;
    uint32_t last_voltage_ms, last_temperature_ms;
    bool config_crc_valid;
    uint8_t expected_config_crc[2]; /* Bytes esperados em 0x0036/0x0037. */
    bool balance_state_known, balance_running;
    uint32_t last_balance_start_ms, last_balance_stop_ms;
} BMS_SlaveData_t;

/* =========================================================================
 * 22 - HANDLE GLOBAL, CONTEXTO PARTILHADO COM CALLBACKS
 * =========================================================================
 * Guardar em memoria persistente: o DMA escreve rx_buf e callbacks usam
 * o endereco deste handle. Nao criar uma copia temporaria para cada leitura.
 * Campos externos/NTC/HV permanecem por compatibilidade, sem nova integracao.
 */
typedef struct
{
    BMS_State_t state;
    uint32_t fault_flags;
    bool ring_intact; /* Ultima verificacao de AMBOS os sentidos; nao deteccao continua. */
    bool ring_using_reverse;
    volatile bool contactor_closed; /* Expectativa LOGICA, nao realimentacao fisica. */
    volatile bool bms_ok;
    BMS_SlaveData_t slave[BMS_NUM_SLAVES];
    uint32_t pack_voltage_mv;
    uint16_t min_cell_mv, max_cell_mv, delta_cell_mv;
    int16_t max_temp_c;
    volatile uint32_t fault_count;
    uint32_t ring_recovery_count, comm_error_count, crc_error_count;
    const char *init_step;          /* Etapa de arranque/POST para localizar uma falha. */
    BMS_Status_t last_error_status; /* Compatibilidade: espelho do contexto LAST. */
    uint8_t last_error_device;      /* 0 bridge; 1/2 endereco; FF Stack/Broadcast. */
    uint16_t last_error_reg;
    BMS_ErrorContext_t first_error, last_error;
    /* Ultimo erro de frame/transporte preservado ANTES de abortar DMA/COMM CLEAR. */
    bool debug_transport_valid;
    BMS_Status_t debug_transport_status;
    uint32_t debug_transport_tick_ms;
    uint32_t debug_uart_error, debug_dma_error;
    uint16_t debug_tx_len, debug_rx_expected, debug_rx_received;
    uint8_t debug_tx[BMS_MAX_FRAME_SIZE];
    uint8_t debug_rx[BMS_RX_BUFFER_SIZE];
    uint8_t bridge_fault_raw[6]; /* Ultima leitura 0x2100..0x2105, para depuracao. */
    bool bridge_fault_valid;
    UART_HandleTypeDef *huart;
    TIM_HandleTypeDef *htim_delay;
    uint32_t inverter_voltage_mv;
    volatile bool precharge_ready;
    bool is_balancing;
    uint32_t balance_cycle_count;
    uint8_t soc_percent; /* Estimativa nao calibrada; consultar soc_valid. */
    uint16_t filtered_mv[BMS_NUM_SLAVES][BMS_CELLS_PER_SLAVE];
    /* Estado do filtro com 16 bits fraccionarios; separado das medicoes de proteccao. */
    uint32_t filtered_q16[BMS_NUM_SLAVES][BMS_CELLS_PER_SLAVE];
    bool filter_primed, post_passed;
    volatile uint32_t nfault_pending, dma_rx_done;
    uint8_t tx_buf[BMS_MAX_FRAME_SIZE];
    uint8_t rx_buf[BMS_RX_BUFFER_SIZE]; /* DMA sempre em memoria persistente. */
    bool addresses_valid, direction_valid, config_verified, timer_valid;
    bool transport_quarantined; /* Abort DMA incerto: impede reutilizar o transporte. */
    volatile uint32_t comm_busy, dma_error, dma_uart_error, dma_hw_error, safety_latched,
        nfault_epoch;
    volatile uint32_t uart_error_pending; /* Erro fora de uma transaccao tambem exige tratamento. */
    bool voltage_valid, temperature_valid, inverter_voltage_valid;
    uint32_t last_voltage_ms, last_temperature_ms, last_inverter_ms;
    bool fault_scan_valid;
    uint32_t last_fault_scan_ms, last_config_check_ms;
    uint32_t last_nfault_diag_ms;
    bool nfault_diag_started;
    uint32_t filtered_pack_mv;
    uint16_t filtered_min_mv, filtered_max_mv, filtered_delta_mv;
    uint32_t warning_flags; /* Avisos recalculados; nao equivalem a fault_flags retidos. */
    bool balance_stop_pending, soc_valid;
    bool physical_safe_confirmed; /* Integracao externa: NAO inferir de uma flag de comando. */
    bool diagnostics_confirmed;   /* Integracao externa: OW/diagnosticos exigidos validados. */
} BMS_Handle_t;

/* Driver de uma unica bridge. Sem chamadas UART concorrentes nem a partir de ISR.
 * Buffer de destino deve ter N bytes (Single) ou 2*N bytes (Stack). */
/* =========================================================================
 * 23 - API PUBLICA
 * =========================================================================
 * Chamadas BQ serializadas no contexto principal. Nao usar UART BQ em ISR.
 * Single Read exige buffer>=N; Stack Read>=2*N.
 * h e o contexto persistente do BMS; nunca copiar durante uma transferencia DMA.
 * dev/device_address e o endereco de comunicacao, nao o indice fisico do slave.
 * reg/first_register e o endereco do primeiro registo; length e numero de bytes.
 * Status descreve a operacao; fault_flags e uma mascara de causas acumuladas.
 * Descricoes completas e efeitos laterais estao junto da implementacao nos .c.
 */
/**
 * @brief Liga o driver aos perifericos e executa o arranque dos BQ.
 *
 * UART4/TIM2 devem estar configurados. O handle deve permanecer em memoria; requer POST posterior.
 */
BMS_Status_t BMS_Init(BMS_Handle_t *h, UART_HandleTypeDef *uart, TIM_HandleTypeDef *timer);

/**
 * @brief Limpa os contextos FIRST/LAST e a captura do ultimo transporte falhado.
 */
void BMS_ClearDebugHistory(BMS_Handle_t *h);

/**
 * @brief Acorda e endereca a rede nos dois sentidos do anel.
 */
BMS_Status_t BMS_AutoAddressing(BMS_Handle_t *h);

/**
 * @brief Configura os dois BQ79616 para 15S e monitorizacao de tensao.
 */
BMS_Status_t BMS_ConfigureSlaves(BMS_Handle_t *h);

/**
 * @brief Confirma os parametros essenciais que os slaves estao a usar.
 */
BMS_Status_t BMS_VerifyConfiguration(BMS_Handle_t *h);

/**
 * @brief Executa um ciclo de monitorizacao dos slaves.
 */
BMS_Status_t BMS_Task_100ms(BMS_Handle_t *h);

/**
 * @brief Adquire as 30 tensoes e publica um conjunto completo.
 */
BMS_Status_t BMS_ReadAllCellVoltages(BMS_Handle_t *h);

/**
 * @brief Le e converte os canais NTC dos dois slaves.
 */
BMS_Status_t BMS_ReadAllTemperatures(BMS_Handle_t *h);

/**
 * @brief Mede a tensao do bus do inversor a partir do GPIO4 do primeiro slave.
 */
BMS_Status_t BMS_ReadInverterVoltage(BMS_Handle_t *h);

/**
 * @brief Compara as amostras com os limites de software existentes.
 */
BMS_Status_t BMS_CheckProtections(BMS_Handle_t *h);

/**
 * @brief Programa temporizadores por celula e confirma o estado do BQ.
 *
 * Endereco activo do slave; bit 0 da mascara = C1. Mascara zero pede paragem.
 */
BMS_Status_t BMS_SetCellBalancing(BMS_Handle_t *h, uint8_t device_address, uint16_t cell_mask);

/**
 * @brief Pede e confirma a paragem nos dois slaves.
 */
BMS_Status_t BMS_StopAllBalancing(BMS_Handle_t *h);

/**
 * @brief Supervisiona as sessoes automaticas de balanceamento.
 */
BMS_Status_t BMS_RunPassiveBalancing(BMS_Handle_t *h);

/**
 * @brief Regista um evento NFAULT sem efectuar UART.
 */
void BMS_NFAULT_IRQHandler(BMS_Handle_t *h);

/**
 * @brief Trata NFAULT pendente ou ainda activo.
 */
BMS_Status_t BMS_ProcessFaults(BMS_Handle_t *h);

/**
 * @brief Le e interpreta falhas da bridge e dos dois slaves.
 *
 * BMS_OK confirma a leitura; consultar tambem *faults e os indicadores de validade.
 */
BMS_Status_t BMS_ScanHardwareFaults(BMS_Handle_t *h, uint32_t *faults);

/**
 * @brief Pede a limpeza dos latches de falha nos BQ.
 */
BMS_Status_t BMS_ClearFaultLatches(BMS_Handle_t *h);

/**
 * @brief Tenta recuperar um caminho DIR1 completo pelos dois slaves.
 */
BMS_Status_t BMS_RingRecovery(BMS_Handle_t *h);

/**
 * @brief Selecciona e verifica um caminho completo pelos dois slaves.
 */
BMS_Status_t BMS_SelectDirection(BMS_Handle_t *h, bool reverse);

/**
 * @brief Passa a FAULT e pede paragem de balanceamento quando necessaria.
 */
void BMS_EmergencyShutdown(BMS_Handle_t *h);

/**
 * @brief Retira permissoes logicas sem fazer comunicacao bloqueante.
 */
void BMS_InhibitOutputs(BMS_Handle_t *h);

/**
 * @brief Consulta NFAULT por nivel fisico, EXTI e evento pendente.
 */
bool BMS_NfaultIsActive(const BMS_Handle_t *h);

/**
 * @brief Retira a expectativa logica de contactor fechado.
 */
void BMS_ContactorOpen(BMS_Handle_t *h);

/**
 * @brief Actualiza a expectativa logica de fecho apos verificar as permissoes.
 */
void BMS_ContactorClose(BMS_Handle_t *h);

/**
 * @brief Envia o ping SHUTDOWN apenas para a bridge.
 */
void BMS_SendShutdownPulse(BMS_Handle_t *h);

//void         BMS_SendHardwareReset(BMS_Handle_t *hbms);   //nao e usada
/**
 * @brief Pede SHUTDOWN da rede; nome historico mantido por compatibilidade.
 *
 * Apesar do nome historico, pede SHUTDOWN, nao uma transicao para SLEEP.
 */
void BMS_EnterSleep(BMS_Handle_t *h);
/* API historica: pede SHUTDOWN completo. */
/**
 * @brief Recalcula as permissoes logicas bms_ok e precharge_ready.
 */
void BMS_UpdateHardwareInterlocks(BMS_Handle_t *h);

/**
 * @brief Refresca o watchdog ja iniciado pela configuracao STM32.
 */
void BMS_IWDG_Refresh(void);

/**
 * @brief Executa as verificacoes basicas antes de monitorizar.
 */
BMS_Status_t BMS_PowerOnSelfTest(BMS_Handle_t *h);

/**
 * @brief Emite COMM CLEAR para recuperar a interface UART.
 */
void BMS_CommClear(BMS_Handle_t *h);

/**
 * @brief Actualiza uma copia IIR para visualizacao.
 */
void BMS_ApplyVoltageFilter(BMS_Handle_t *h);

/**
 * @brief Estima uma percentagem de carga por interpolacao da curva OCV do projecto.
 *
 * Media de uma celula em mV. Estimativa OCV nao calibrada; nao prova carga disponivel.
 */
uint8_t BMS_EstimateSoC(uint16_t average_cell_mv);

/**
 * @brief Escreve de um a oito registos de um unico dispositivo.
 */
BMS_Status_t BMS_WriteSingleDevice(BMS_Handle_t *h,
                                   uint8_t device_address,
                                   uint16_t first_register,
                                   const uint8_t *data,
                                   uint8_t length);

/**
 * @brief Le de 1 a 128 bytes de um dispositivo.
 *
 * Reservar length bytes; so publica o buffer depois de validar a resposta completa.
 */
BMS_Status_t BMS_ReadSingleDevice(BMS_Handle_t *h,
                                  uint8_t device_address,
                                  uint16_t first_register,
                                  uint8_t *data,
                                  uint8_t length);

/**
 * @brief Envia uma escrita Broadcast normal.
 */
BMS_Status_t
BMS_WriteBroadcast(BMS_Handle_t *h, uint16_t first_register, const uint8_t *data, uint8_t length);

/**
 * @brief Envia a mesma escrita aos dispositivos da stack.
 */
BMS_Status_t
BMS_WriteStack(BMS_Handle_t *h, uint16_t first_register, const uint8_t *data, uint8_t length);

/**
 * @brief Le o mesmo bloco dos dois slaves.
 *
 * Reservar 2 * length_per_slave bytes. Ordem de saida: S1 e depois S2, independentemente de DIR.
 */
BMS_Status_t
BMS_ReadStack(BMS_Handle_t *h, uint16_t first_register, uint8_t *data, uint8_t length_per_slave);

/**
 * @brief Muda DIR_SEL dos slaves pela porta oposta.
 */
BMS_Status_t BMS_WriteBroadcastReverse(
    BMS_Handle_t *h, uint16_t first_register, uint8_t init, const uint8_t *data, uint8_t length);

/**
 * @brief Calcula o CRC utilizado nos frames BQ e na imagem de configuracao.
 */
uint16_t BMS_CalculateCRC16(const uint8_t *data, uint16_t length);

/**
 * @brief Escolhe o endereco de uma identidade fisica no sentido actual.
 *
 * physical_index: 0=S1, 1=S2. Endereco invalido devolve 0xFF.
 */
uint8_t BMS_GetSlaveAddress(const BMS_Handle_t *h, uint8_t physical_index);

/**
 * @brief Verifica se as amostras existem e ainda sao recentes.
 *
 * Tensoes sao sempre exigidas; os dois booleanos acrescentam os requisitos de NTC e HV.
 */
bool BMS_MeasurementsFresh(const BMS_Handle_t *h, bool require_temperature, bool require_hv);

/* Estado de AQUISICAO dos slaves, separado de autorizacao do veiculo.
 * Nao confirma NTC, corrente, diagnostico OW ou actuacao de potencia. */
/**
 * @brief Indica aquisicao de tensao validada dos dois slaves.
 */
bool BMS_SlavesReady(const BMS_Handle_t *h);

/**
 * @brief Converte o resultado de uma operacao em texto.
 */
const char *BMS_GetStatusString(BMS_Status_t status);

/**
 * @brief Converte o estado do driver em texto de debug.
 */
const char *BMS_GetStateString(BMS_State_t state);

/**
 * @brief Devolve uma descricao principal para o conjunto de falhas.
 *
 * Devolve apenas a categoria prioritaria; consultar a mascara completa para falhas simultaneas.
 */
const char *BMS_GetFaultString(uint32_t faults);

/**
 * @brief Converte um codigo de retorno numa categoria de falha do BMS.
 */
uint32_t BMS_FaultForStatus(BMS_Status_t status);

/* Contexto principal; reason/step sao literais estaticos, nunca buffers locais. */
/**
 * @brief Guarda a primeira causa e a causa mais recente de uma falha.
 *
 * Contexto principal. reason deve ter duracao estatica; os campos expected/observed/mask contextualizam a causa.
 */
BMS_Status_t BMS_RecordError(BMS_Handle_t *h,
                             BMS_Status_t status,
                             const char *reason,
                             uint8_t dev,
                             uint16_t reg,
                             uint32_t expected,
                             uint32_t observed,
                             uint32_t mask);

/**
 * @brief Espera em milissegundos pela base temporal HAL.
 *
 * Espera bloqueante; requer tick HAL activo e nao refresca o watchdog.
 */
void BMS_DelayMs(uint32_t milliseconds);

/**
 * @brief Espera em microssegundos usando TIM2 em roda livre.
 *
 * Espera bloqueante com TIM2 a 1 MHz; limite aceite de 1000000 us.
 */
void BMS_DelayUs(BMS_Handle_t *h, uint32_t microseconds);

#endif
