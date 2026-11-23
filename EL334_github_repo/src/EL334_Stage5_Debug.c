/*=============================================================================
 *  EL334 Mini-Project - Stage 5 Fault Isolation
 *  FRDM-K64F + Multi-Function Shield
 *
 *  Use this when Stage 5 freezes. It splits Stage 5 into three smaller tests
 *  so you can tell WHICH part stops working.
 *
 *  Run them in this order: 51 -> 52 -> 53
 *
 *    51 : PIT1 only, no display.      LED1 must blink forever at 1 Hz.
 *    52 : PIT0 only, no counting.     7-segment must show 1234 forever.
 *    53 : Both + main-loop heartbeat. LED4 flickers while main() is alive.
 *
 *  A HardFault handler is included: if the CPU faults, ALL FOUR LEDs turn
 *  on solid and stay on. That tells you it crashed, without a debugger.
 *
 *  Keep CLOCK_SETUP = 1 in system_MK64F12.c for these tests.
 *===========================================================================*/

#include "MK64F12.h"
#include <stdint.h>

#define TEST_STAGE   51          /* 51, 52 or 53 */

#define BUS_CLK_HZ   60000000u   /* CLOCK_SETUP 1 -> core 120 MHz, bus 60 MHz */

/*--------------------------- Pin definitions -------------------------------*/
#define LED1_PIN    1u      /* PTD1  - D13 */
#define LED2_PIN    3u      /* PTD3  - D12 */
#define LED3_PIN    2u      /* PTD2  - D11 */
#define LED4_PIN    0u      /* PTD0  - D10 */
#define LED_ALL     ((1u<<LED1_PIN)|(1u<<LED2_PIN)|(1u<<LED3_PIN)|(1u<<LED4_PIN))

#define LATCH_PIN   23u     /* PTB23 - D4 */
#define CLK_PIN     3u      /* PTC3  - D7 */
#define DATA_PIN    12u     /* PTC12 - D8 */

/*--------------------------- Shared state ----------------------------------*/
static const uint8_t SEGMENTS[10] = {
    0xC0, 0xF9, 0xA4, 0xB0, 0x99, 0x92, 0x82, 0xF8, 0x80, 0x90
};
static const uint8_t DIGIT_SELECT[4] = { 0xF1, 0xF2, 0xF4, 0xF8 };

static volatile uint8_t  g_disp[4]  = {1, 2, 3, 4};
static volatile uint32_t g_tick_ms  = 0;
static volatile uint32_t g_pit0_cnt = 0;    /* watch these two in the debugger */
static volatile uint32_t g_pit1_cnt = 0;

/*===========================================================================
 *  HardFault catcher - all LEDs ON means the CPU crashed
 *===========================================================================*/
void HardFault_Handler(void)
{
    SIM->SCGC5 |= SIM_SCGC5_PORTD_MASK;
    PORTD->PCR[LED1_PIN] = PORT_PCR_MUX(1);
    PORTD->PCR[LED2_PIN] = PORT_PCR_MUX(1);
    PORTD->PCR[LED3_PIN] = PORT_PCR_MUX(1);
    PORTD->PCR[LED4_PIN] = PORT_PCR_MUX(1);
    PTD->PDDR |= LED_ALL;
    PTD->PCOR  = LED_ALL;           /* active low -> all ON */
    for (;;) { }
}

/*===========================================================================
 *  LED
 *===========================================================================*/
static void LED_Init(void)
{
    SIM->SCGC5 |= SIM_SCGC5_PORTD_MASK;
    PORTD->PCR[LED1_PIN] = PORT_PCR_MUX(1);
    PORTD->PCR[LED2_PIN] = PORT_PCR_MUX(1);
    PORTD->PCR[LED3_PIN] = PORT_PCR_MUX(1);
    PORTD->PCR[LED4_PIN] = PORT_PCR_MUX(1);
    PTD->PDDR |= LED_ALL;
    PTD->PSOR  = LED_ALL;           /* all OFF */
}

static void LED1_Toggle(void) { PTD->PTOR = (1u << LED1_PIN); }
static void LED4_Toggle(void) { PTD->PTOR = (1u << LED4_PIN); }

/*===========================================================================
 *  7-Segment
 *===========================================================================*/
static void SevenSeg_Init(void)
{
    SIM->SCGC5 |= SIM_SCGC5_PORTB_MASK | SIM_SCGC5_PORTC_MASK;
    PORTB->PCR[LATCH_PIN] = PORT_PCR_MUX(1);
    PORTC->PCR[CLK_PIN]   = PORT_PCR_MUX(1);
    PORTC->PCR[DATA_PIN]  = PORT_PCR_MUX(1);
    PTB->PDDR |= (1u << LATCH_PIN);
    PTC->PDDR |= (1u << CLK_PIN) | (1u << DATA_PIN);
    PTB->PCOR  = (1u << LATCH_PIN);
    PTC->PCOR  = (1u << CLK_PIN);
}

static void SevenSeg_ShiftByte(uint8_t b)
{
    int8_t i;
    for (i = 7; i >= 0; i--) {
        if (b & (1u << i)) PTC->PSOR = (1u << DATA_PIN);
        else               PTC->PCOR = (1u << DATA_PIN);
        PTC->PSOR = (1u << CLK_PIN);
        PTC->PCOR = (1u << CLK_PIN);
    }
}

static void SevenSeg_ShowDigit(uint8_t pos, uint8_t numeral)
{
    if (numeral > 9u) numeral = 0u;             /* bounds guard */
    PTB->PCOR = (1u << LATCH_PIN);
    SevenSeg_ShiftByte(SEGMENTS[numeral]);
    SevenSeg_ShiftByte(DIGIT_SELECT[pos & 0x03u]);
    PTB->PSOR = (1u << LATCH_PIN);
}

static void SevenSeg_SetNumber(uint16_t v)
{
    if (v > 9999u) v = 9999u;
    g_disp[0] = (uint8_t)((v / 1000u) % 10u);
    g_disp[1] = (uint8_t)((v / 100u)  % 10u);
    g_disp[2] = (uint8_t)((v / 10u)   % 10u);
    g_disp[3] = (uint8_t)( v          % 10u);
}

/*===========================================================================
 *  PIT - note the __DSB() after clearing the flag.
 *  Without it the write that clears TIF can still be in the write buffer
 *  when the ISR returns, so the core may re-enter the handler immediately.
 *  This gets more likely as the core clock rises relative to the bus clock,
 *  which is exactly what happened when we went from 21 MHz to 120 MHz.
 *===========================================================================*/
#define PIT_LDVAL_MS(ms)   (((BUS_CLK_HZ / 1000u) * (ms)) - 1u)

void PIT0_IRQHandler(void)
{
    static uint8_t pos = 0u;

    PIT->CHANNEL[0].TFLG = PIT_TFLG_TIF_MASK;
    __DSB();                                    /* flush the flag write */

    g_pit0_cnt++;
    SevenSeg_ShowDigit(pos, g_disp[pos]);
    pos = (uint8_t)((pos + 1u) & 0x03u);
}

void PIT1_IRQHandler(void)
{
    PIT->CHANNEL[1].TFLG = PIT_TFLG_TIF_MASK;
    __DSB();

    g_pit1_cnt++;
    g_tick_ms++;
}

static void PIT_Start(uint8_t use_ch0, uint8_t use_ch1)
{
    SIM->SCGC6 |= SIM_SCGC6_PIT_MASK;
    PIT->MCR = 0u;

    if (use_ch0) {
        PIT->CHANNEL[0].LDVAL = PIT_LDVAL_MS(2);
        PIT->CHANNEL[0].TCTRL = PIT_TCTRL_TIE_MASK | PIT_TCTRL_TEN_MASK;
        NVIC_SetPriority(PIT0_IRQn, 2);
        NVIC_EnableIRQ(PIT0_IRQn);
    }
    if (use_ch1) {
        PIT->CHANNEL[1].LDVAL = PIT_LDVAL_MS(1);
        PIT->CHANNEL[1].TCTRL = PIT_TCTRL_TIE_MASK | PIT_TCTRL_TEN_MASK;
        NVIC_SetPriority(PIT1_IRQn, 2);
        NVIC_EnableIRQ(PIT1_IRQn);
    }
}

/*===========================================================================
 *  MAIN
 *===========================================================================*/
int main(void)
{
/*---------------------------------------------------------------------------
 *  51 : PIT1 alone. No display, no 7-segment pins touched.
 *       PASS = LED1 blinks forever, 0.5 s on / 0.5 s off.
 *       FAIL = stops blinking  -> problem is the clock, PIT or NVIC setup
 *       FAIL = all 4 LEDs on   -> HardFault
 *-------------------------------------------------------------------------*/
#if (TEST_STAGE == 51)
    uint32_t next = 500u;

    LED_Init();
    PIT_Start(0, 1);

    for (;;) {
        if (g_tick_ms >= next) {
            next += 500u;
            LED1_Toggle();
        }
    }

/*---------------------------------------------------------------------------
 *  52 : PIT0 alone driving the display. Nothing counts.
 *       PASS = 7-segment shows 1234 steadily, forever, no flicker
 *       FAIL = freezes or garbles -> problem is the display ISR
 *-------------------------------------------------------------------------*/
#elif (TEST_STAGE == 52)
    LED_Init();
    SevenSeg_Init();
    g_disp[0] = 1; g_disp[1] = 2; g_disp[2] = 3; g_disp[3] = 4;
    PIT_Start(1, 0);

    for (;;) {
        __NOP();
    }

/*---------------------------------------------------------------------------
 *  53 : Everything together, plus a main-loop heartbeat on LED4.
 *       LED4 flickers fast for as long as main() keeps looping.
 *
 *       7-seg counts AND LED4 flickers   -> all good
 *       7-seg frozen BUT LED4 flickers   -> main() alive, timer stopped
 *       7-seg frozen AND LED4 frozen     -> main() stopped or stuck in an ISR
 *       all 4 LEDs solid on              -> HardFault
 *-------------------------------------------------------------------------*/
#elif (TEST_STAGE == 53)
    uint32_t next_sec = 1000u;
    uint32_t hb = 0u;
    uint16_t seconds = 0u;

    LED_Init();
    SevenSeg_Init();
    PIT_Start(1, 1);

    for (;;) {
        if (++hb >= 300000u) { hb = 0u; LED4_Toggle(); }   /* heartbeat */

        if (g_tick_ms >= next_sec) {
            next_sec += 1000u;
            seconds++;
            SevenSeg_SetNumber(seconds);
        }
    }

#else
    #error "TEST_STAGE must be 51, 52 or 53"
#endif
}
