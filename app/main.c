#include "mik32_hal_usart.h"
#include "mik32_hal_i2c.h"
#include "mik32_hal_irq.h"
#include "mik32_hal_dma.h"
#include "mik32_hal_adc.h"
#include "mik32_hal_timer32.h"
#include "string.h"
#include "stdlib.h"
#include "xprintf.h"

#define BLOCK_SIZE          1024u
#define SAMPLE_RATE_HZ      10000u
#define SYSCLK_HZ           32000000u
#define TIMER_TOP           ((SYSCLK_HZ / SAMPLE_RATE_HZ) - 1u)

#define USART_BAUD          921600u
#define USART_TIMEOUT       0

#define PACKET_SYNC0        0xA5
#define PACKET_SYNC1        0x5A

static volatile bool adc_busy = false;
static volatile uint32_t block_seq = 0;
static volatile uint32_t dropped_triggers = 0;
static volatile uint32_t tx_errors = 0;

static void SystemClock_Config();
static void USART_Init();
static void DMA_Init();
static void ADC_Init(void);
static void Timer32_Init(void);

static void configure_interrupts();
static void configure_mem_to_uart_dma(DMA_InitTypeDef*, DMA_ChannelHandleTypeDef*);

ADC_HandleTypeDef    hadc;
TIMER32_HandleTypeDef htimer32;
USART_HandleTypeDef husart0;
DMA_InitTypeDef hdma;

DMA_ChannelHandleTypeDef hdma_ch_mem_to_uart;


int main()
{
    SystemClock_Config();
    USART_Init();
    DMA_Init();
    ADC_Init();
    Timer32_Init();

    configure_interrupts();
    configure_mem_to_uart_dma(&hdma, &hdma_ch_mem_to_uart);

    HAL_Timer32_Value_Clear(&htimer32);
    HAL_Timer32_Start(&htimer32);
    HAL_USART_Print(&husart0, "start\n\r", 100);

    while (1)
    {
        if(true) {
            //HAL_DMA_Start(&hdma_ch_mem_to_uart, top.byteArray, (void*)&(husart0.Instance->TXDATA), top.length - 1);
        }
    }
}

void trap_handler(void)
{
    if (EPIC_CHECK_TIMER32_0())
    {
        //HAL_TIMER32_INTERRUPTFLAGS_CLEAR(&htimer32);
        if (!adc_busy)
        {
            adc_busy = true;
            HAL_ADC_Single(&hadc);
        }
        else
        {
            dropped_triggers++;
        }
        TIMER32_0->INT_CLEAR = TIMER32_INT_OVERFLOW_M;
        EPIC->CLEAR = EPIC_LINE_TIMER32_0_S;
    }

    if (EPIC_CHECK_ADC())
    {
        uint16_t sample = HAL_ADC_GetValue(&hadc);
        adc_busy = false;
        //HAL_USART_Print(&husart0, "ADC int\n\r", 100);
        xprintf("ADC: %04d/%d\r\n", sample, 4096);
        HAL_EPIC_Clear(HAL_EPIC_ADC_MASK);

    }
}


void SystemClock_Config(void)
{
    PCC_InitTypeDef PCC_OscInit = {0};

    PCC_OscInit.OscillatorEnable = PCC_OSCILLATORTYPE_ALL;
    PCC_OscInit.FreqMon.OscillatorSystem = PCC_OSCILLATORTYPE_OSC32M;
    PCC_OscInit.FreqMon.ForceOscSys = PCC_FORCE_OSC_SYS_UNFIXED;
    PCC_OscInit.FreqMon.Force32KClk = PCC_FREQ_MONITOR_SOURCE_OSC32K;
    PCC_OscInit.AHBDivider = 0;
    PCC_OscInit.APBMDivider = 0;
    PCC_OscInit.APBPDivider = 0;
    PCC_OscInit.HSI32MCalibrationValue = 128;
    PCC_OscInit.LSI32KCalibrationValue = 8;
    PCC_OscInit.RTCClockSelection = PCC_RTC_CLOCK_SOURCE_AUTO;
    PCC_OscInit.RTCClockCPUSelection = PCC_CPU_RTC_CLOCK_SOURCE_OSC32K;
    HAL_PCC_Config(&PCC_OscInit);
}

static void Timer32_Init(void)
{
        // Включение тактирования TIMER32_0
    PM->CLK_APB_M_SET = PM_CLOCK_APB_M_TIMER32_0_M;
    TIMER32_0->ENABLE = 0;
    TIMER32_0->TOP = TIMER_TOP;
    TIMER32_0->PRESCALER = 0;
    TIMER32_0->CONTROL =
        TIMER32_CONTROL_MODE_UP_M | TIMER32_CONTROL_CLOCK_PRESCALER_M;
    TIMER32_0->INT_MASK = 0;
    TIMER32_0->INT_CLEAR = 0xFFFFFFFF;
    TIMER32_0->ENABLE = 1;
    // Включение прерывания по переполнению
    TIMER32_0->INT_MASK = TIMER32_INT_OVERFLOW_M;

}

static void ADC_Init(void) {
    hadc.Instance = ANALOG_REG;
    hadc.Init.EXTRef = ADC_EXTREF_OFF;
    hadc.Init.EXTClb = ADC_EXTCLB_CLBREF;

    HAL_ADC_Init(&hadc);
}


void DMA_Init(void)
{
    hdma.Instance = DMA_CONFIG;
    hdma.CurrentValue = DMA_CURRENT_VALUE_ENABLE;
    HAL_DMA_Init(&hdma);
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
    husart0.baudrate = USART_BAUD;

    husart0.dma_tx_request = Enable;
    husart0.dma_rx_request = Disable;

    HAL_USART_Init(&husart0);
}

void configure_interrupts() {
    __HAL_PCC_EPIC_CLK_ENABLE();
    HAL_EPIC_MaskLevelSet(HAL_EPIC_UART_0_MASK | HAL_EPIC_TIMER32_1_MASK);
    HAL_USART_RXNE_EnableInterrupt(&husart0);
    //HAL_USART_TXC_EnableInterrupt(&husart0);
    HAL_EPIC_MaskEdgeSet(HAL_EPIC_ADC_MASK);
    TIMER32_0->INT_MASK = TIMER32_INT_OVERFLOW_M;

    HAL_IRQ_EnableInterrupts();
}

void configure_mem_to_uart_dma(DMA_InitTypeDef* hdma, DMA_ChannelHandleTypeDef* ch) {
    ch->dma = hdma;

    /* Настройки канала */
    ch->ChannelInit.Channel = DMA_CHANNEL_0;
    ch->ChannelInit.Priority = DMA_CHANNEL_PRIORITY_VERY_HIGH;

    ch->ChannelInit.ReadMode = DMA_CHANNEL_MODE_MEMORY;
    ch->ChannelInit.ReadInc = DMA_CHANNEL_INC_ENABLE;
    ch->ChannelInit.ReadSize = DMA_CHANNEL_SIZE_BYTE; /* data_len должно быть кратно read_size */
    ch->ChannelInit.ReadBurstSize = 0;                /* read_burst_size должно быть кратно read_size */
    ch->ChannelInit.ReadRequest = DMA_CHANNEL_USART_0_REQUEST;
    ch->ChannelInit.ReadAck = DMA_CHANNEL_ACK_DISABLE;

    ch->ChannelInit.WriteMode = DMA_CHANNEL_MODE_PERIPHERY;
    ch->ChannelInit.WriteInc = DMA_CHANNEL_INC_DISABLE;
    ch->ChannelInit.WriteSize = DMA_CHANNEL_SIZE_BYTE; /* data_len должно быть кратно write_size */
    ch->ChannelInit.WriteBurstSize = 0;                /* write_burst_size должно быть кратно read_size */
    ch->ChannelInit.WriteRequest = DMA_CHANNEL_USART_0_REQUEST;
    ch->ChannelInit.WriteAck = DMA_CHANNEL_ACK_ENABLE;
}
