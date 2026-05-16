#include "epic.h"
#include "gpio.h"
#include "mik32_memory_map.h"
#include "pad_config.h"
#include "power_manager.h"
#include "riscv-irq.h"
#include "scr1_timer.h"
#include "timer32.h"

#include "mik32_hal.h"
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#include "mik32_hal_irq.h"
#include "mik32_hal_adc.h"
#include "mik32_hal_usart.h"
#include "xprintf.h"

#define BLOCK_SIZE          1024u
#define SAMPLE_RATE_HZ      10000u
#define SYSCLK_HZ           32000000u
#define TIMER_TOP           ((SYSCLK_HZ / SAMPLE_RATE_HZ) - 1u)

#define USART_BAUD          921600u
#define USART_TIMEOUT       0

static uint16_t adc_buf[2][BLOCK_SIZE];
static volatile bool buf_ready[2] = {false, false};
static volatile uint8_t fill_buf = 0;
static volatile uint16_t fill_pos = 0;
static volatile bool adc_busy = false;

static volatile uint32_t block_seq = 0;
static volatile uint32_t dropped_triggers = 0;
static volatile uint32_t tx_errors = 0;

void SystemClock_Config();
static void USART_Init();
void GPIO_Init();
void TMR_Init();
static void ADC_Init();
void EPIC_trap_handler();
void exception_trap_handler();
void process_and_send_block(uint8_t idx);

static void configure_interrupts();

void exception_trap_handler() {
  while (1) {
  }
  return;
}

USART_HandleTypeDef husart0;
ADC_HandleTypeDef    hadc;

void EPIC_trap_handler() {
    if (EPIC_CHECK_TIMER32_0())
    {
        uint16_t sample = HAL_ADC_GetValue(&hadc);
        adc_buf[fill_buf][fill_pos++] = sample;
        if (fill_pos >= BLOCK_SIZE)
        {
            buf_ready[fill_buf] = true;
            fill_buf ^= 1u;
            fill_pos = 0;
        }
        TIMER32_0->INT_CLEAR = TIMER32_INT_OVERFLOW_M;
        EPIC->CLEAR = EPIC_LINE_TIMER32_0_S;
    }
}

int main()
{
    SystemClock_Config();
    GPIO_Init();
    TMR_Init();
    USART_Init();
    ADC_Init();

    // Включение тактирования EPIC
    PM->CLK_APB_M_SET = PM_CLOCK_APB_M_EPIC_M;
    // Включение прерываний от TIMER32_0
    EPIC->MASK_LEVEL_SET = 1 << (EPIC_LINE_TIMER32_0_S);

    riscv_irq_set_handler(RISCV_IRQ_MEI, EPIC_trap_handler);
    riscv_exception_set_handler(RISCV_EXCP_LOAD_ADDRESS_MISALIGNED,
                                exception_trap_handler);
    riscv_irq_enable(RISCV_IRQ_MEI);
    //HAL_EPIC_MaskEdgeSet(HAL_EPIC_ADC_MASK);
    riscv_irq_global_enable();

    while (1) {
        if (buf_ready[0])
        {
            buf_ready[0] = false;
            process_and_send_block(0);
        }
        else if (buf_ready[1])
        {
            buf_ready[1] = false;
            process_and_send_block(1);
        }
    }
}

void process_and_send_block(uint8_t idx)
{
    int32_t sum = 0;
    int16_t min_v = INT16_MAX;
    int16_t max_v = INT16_MIN;

    for (uint16_t i = 0; i < BLOCK_SIZE; ++i)
        sum += adc_buf[idx][i];

    uint16_t mean = (uint16_t)(sum / (int32_t)BLOCK_SIZE);

    for (uint16_t i = 0; i < BLOCK_SIZE; ++i)
    {
        int16_t centered = (int16_t)adc_buf[idx][i];

        if (centered < min_v) min_v = centered;
        if (centered > max_v) max_v = centered;
    }
    xprintf("Mean: %d ; Min: %d ; Max: %d\n\r", mean, min_v, max_v);
}

void TMR_Init()
{
    PM->CLK_APB_M_SET = PM_CLOCK_APB_M_TIMER32_0_M;
    TIMER32_0->ENABLE = 0;
    TIMER32_0->TOP = TIMER_TOP;
    TIMER32_0->PRESCALER = 0;
    TIMER32_0->CONTROL =
        TIMER32_CONTROL_MODE_UP_M | TIMER32_CONTROL_CLOCK_PRESCALER_M;
    TIMER32_0->INT_MASK = 0;
    TIMER32_0->INT_CLEAR = 0xFFFFFFFF;
    TIMER32_0->ENABLE = 1;
    TIMER32_0->INT_MASK = TIMER32_INT_OVERFLOW_M;
}

void SystemClock_Config(void)
{
    WU->CLOCKS_SYS &=
        ~(0b11 << WU_CLOCKS_SYS_OSC32M_EN_S); // Включить OSC32M и HSI32M
    WU->CLOCKS_BU &=
        ~(0b11 << WU_CLOCKS_BU_OSC32K_EN_S); // Включить OSC32K и LSI32K

    // Поправочный коэффициент HSI32M
    WU->CLOCKS_SYS = (WU->CLOCKS_SYS & (~WU_CLOCKS_SYS_ADJ_HSI32M_M)) |
                     WU_CLOCKS_SYS_ADJ_HSI32M(128);
    // Поправочный коэффициент LSI32K
    WU->CLOCKS_BU = (WU->CLOCKS_BU & (~WU_CLOCKS_BU_ADJ_LSI32K_M)) |
                    WU_CLOCKS_BU_ADJ_LSI32K(8);

    // Автоматический выбор источника опорного тактирования
    WU->CLOCKS_SYS &= ~WU_CLOCKS_SYS_FORCE_32K_CLK_M;

    // ожидание готовности
    while (!(PM->FREQ_STATUS & PM_FREQ_STATUS_OSC32M_M))
      ;

    // переключение на тактирование от OSC32M
    PM->AHB_CLK_MUX = PM_AHB_CLK_MUX_OSC32M_M | PM_AHB_FORCE_MUX_UNFIXED;
    PM->DIV_AHB = 0;   // Задать делитель шины AHB.
    PM->DIV_APB_M = 0; // Задать делитель шины APB_M.
    PM->DIV_APB_P = 0; // Задать делитель шины APB_P.
}

static void ADC_Init() {
    hadc.Instance = ANALOG_REG;
    hadc.Init.EXTRef = ADC_EXTREF_OFF;
    hadc.Init.EXTClb = ADC_EXTCLB_CLBREF;

    HAL_ADC_Init(&hadc);
    HAL_ADC_ContinuousEnable(&hadc);
}

void GPIO_Init()
{
    PM->CLK_APB_P_SET = PM_CLOCK_APB_P_GPIO_0_M;
    PM->CLK_APB_P_SET = PM_CLOCK_APB_P_GPIO_1_M;
    PM->CLK_APB_P_SET = PM_CLOCK_APB_P_GPIO_2_M;
    PM->CLK_APB_P_SET = PM_CLOCK_APB_P_GPIO_IRQ_M;
}

void USART_Init()
{
    husart0.Instance = UART_0;
    husart0.transmitting = Enable;
    husart0.receiving = Enable;
    husart0.frame = Frame_8bit;
    husart0.parity_bit = Disable;
    husart0.parity_bit_inversion = Disable;
    husart0.bit_direction = LSB_First;
    husart0.data_inversion = Disable;
    husart0.tx_inversion = Disable;
    husart0.rx_inversion = Disable;
    husart0.swap = Disable;
    husart0.lbm = Disable;
    husart0.stop_bit = StopBit_1;
    husart0.mode = Asynchronous_Mode;
    husart0.xck_mode = XCK_Mode3;
    husart0.last_byte_clock = Disable;
    husart0.overwrite = Disable;
    husart0.rts_mode = AlwaysEnable_mode;
    husart0.channel_mode = Duplex_Mode;
    husart0.tx_break_mode = Disable;
    husart0.Interrupt.ctsie = Disable;
    husart0.Interrupt.eie = Disable;
    husart0.Interrupt.idleie = Disable;
    husart0.Interrupt.lbdie = Disable;
    husart0.Interrupt.peie = Disable;
    husart0.Interrupt.rxneie = Disable;
    husart0.Interrupt.tcie = Disable;
    husart0.Interrupt.txeie = Disable;
    husart0.Modem.rts = Disable; //out
    husart0.Modem.cts = Disable; //in
    husart0.Modem.dtr = Disable; //out
    husart0.Modem.dcd = Disable; //in
    husart0.Modem.dsr = Disable; //in
    husart0.Modem.ri = Disable;  //in
    husart0.Modem.ddis = Disable;//out
    husart0.baudrate = 921600;

    husart0.dma_tx_request = Disable;
    husart0.dma_rx_request = Disable;

    HAL_USART_Init(&husart0);
}
