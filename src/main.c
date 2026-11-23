/*=============================================================================
 *  EL334 Microprocessor and Microcontroller System Laboratory  (1/69)
 *  Mini-Project : Smart Security / Access Station
 *
 *  Platform : NXP FRDM-K64F + Arduino Multi-Function Shield
 *  Toolchain: Keil uVision 5 (ARM Compiler), CMSIS register-level access
 *
 *---------------------------------------------------------------------------
 *  SYSTEM OVERVIEW
 *---------------------------------------------------------------------------
 *  A six-state home security controller.
 *
 *      DISARMED --[S1]--> ARMING --[10 s]--> ARMED
 *          ^                                   |
 *          |                         [PIR motion detected]
 *          |                                   v
 *       UNLOCK <--[correct code]-- ALARM <--[15 s]-- TRIGGERED
 *          ^                         ^                  |
 *          +--[correct code]---------+------------------+
 *
 *  While in TRIGGERED or ALARM the user disarms the system by entering a
 *  four-key secret code on S1/S2/S3. UNLOCK shows the confirmation for
 *  1.5 s and then returns to DISARMED.
 *
 *---------------------------------------------------------------------------
 *  REQUIRED FEATURES (per assignment)
 *---------------------------------------------------------------------------
 *  Timer / Counter : PIT channel 0  - 2 ms  7-segment multiplex refresh
 *                    PIT channel 1  - 1 ms  system tick + switch debounce
 *                    FTM0 channel 6 - PWM tone generation for the buzzer
 *  Interrupt       : PORTB vector   - S1, S2, S3
 *                    PORTC vector   - PIR sensor
 *                    PIT0 / PIT1 vectors
 *  Modular code    : every device has its own group of helper functions.
 *                    main() only initialises and dispatches.
 *
 *---------------------------------------------------------------------------
 *  PIN MAP   (verified against the MFS schematic and the K64F Arduino header)
 *---------------------------------------------------------------------------
 *  Device              Arduino   K64F      Notes
 *  LED1 (silk D1)      D13       PTD1      active LOW
 *  LED2 (silk D2)      D12       PTD3      active LOW
 *  LED3 (silk D3)      D11       PTD2      active LOW
 *  LED4 (silk D4)      D10       PTD0      active LOW
 *  Switch S1           A1        PTB3      active LOW, internal pull-up
 *  Switch S2           A2        PTB10     active LOW, internal pull-up
 *  Switch S3           A3        PTB11     active LOW, internal pull-up
 *  Buzzer              D3        PTA1      PNP driver, active LOW / FTM0_CH6
 *  7-seg LATCH         D4        PTB23     74HC595 LTCHCLK
 *  7-seg CLK           D7        PTC3      74HC595 SFTCLK
 *  7-seg DATA          D8        PTC12     74HC595 SDI
 *  PIR HC-SR501 OUT    D6        PTC2      rising-edge interrupt
 *  LCD I2C (PCF8574)   D0/D1     PTC16/17  software I2C, APC220 header
 *
 *---------------------------------------------------------------------------
 *  HARDWARE NOTES  (found during bring-up, see the report)
 *---------------------------------------------------------------------------
 *  1. Shield jumper J2 must be REMOVED. It ties the 10 k switch pull-ups to
 *     +5 V, which the 3.3 V K64F pins are not tolerant of. The MCU internal
 *     pull-ups are used instead.
 *  2. The buzzer is driven by a PNP transistor whose emitter sits at +5 V.
 *     Driving the pin HIGH only reaches 3.3 V, leaving Veb = 1.7 V, so the
 *     transistor keeps conducting. The pin is therefore switched to an INPUT
 *     (high-Z) to silence the buzzer. See Buzzer_NoTone().
 *  3. The shield potentiometer and the DS18B20 socket are pulled to +5 V and
 *     must not be connected to a K64F analog pin without a divider.
 *  4. The PIR uses the shield's 3-pin external-device block (bottom right,
 *     12 pins as 3 columns x 4 rows). Columns, left to right: GND, +5 V,
 *     signal. Rows, top to bottom: D5, D6, D9, A5. The PIR's OUT goes to the
 *     signal pin of the D6 row. D5 is avoided because PTA2 is the JTAG/SWO
 *     trace pin, and A5 is left free for an analog sensor.
 *  5. The shield covers D14/D15, so the I2C LCD is plugged into the shield's
 *     APC220 header instead: pin 1 GND, pin 2 +5 V, pin 4 D1, pin 5 D0.
 *     D0/D1 = PTC16/PTC17 have no hardware I2C function, so the LCD module
 *     drives the bus in software. A line is never driven HIGH: a "1" turns
 *     the pin into an input and the backpack's own pull-up raises it. PTC16
 *     and PTC17 are digital-only pins rated to 5.5 V (K64 data sheet, VDIO).
 *     The SDA/SCL order and the backpack address are detected at start-up
 *     and shown on the 7-segment display for 1.5 s:
 *     "L 27" or "L 3F" = address that answered, "L --" = no LCD found.
 *
 *  Comments are written in English on purpose: the Keil editor defaults to
 *  ANSI encoding and corrupts UTF-8 Thai text when it saves the file.
 *===========================================================================*/

#include "MK64F12.h"
#include <stdint.h>

/*===========================================================================
 *  1. BUILD CONFIGURATION
 *===========================================================================*/

/* 0 = no PIR fitted yet, switch S3 simulates a motion event while ARMED
 * 1 = real HC-SR501 on the shield's D6 header, signal reaching PTC2        */
#define PIR_USE_HARDWARE    1

/* 0 = no LCD fitted (calls become no-ops)
 * 1 = 16x2 I2C LCD (PCF8574 backpack) on the shield's APC220 header,
 *     software I2C on D0 = PTC16 and D1 = PTC17 (hardware note 5)          */
#define USE_LCD             1

/* Bus clock. system_MK64F12.c uses CLOCK_SETUP 0 (FEI mode).
 * Calibrated by timing the Stage-5 second counter against a stopwatch.
 * If CLOCK_SETUP is changed to 1 (PEE), use 60000000u here instead.        */
#define BUS_CLK_HZ          20485760u

/*===========================================================================
 *  2. PIN DEFINITIONS
 *===========================================================================*/
#define LED1_PIN    1u          /* PTD1 */
#define LED2_PIN    3u          /* PTD3 */
#define LED3_PIN    2u          /* PTD2 */
#define LED4_PIN    0u          /* PTD0 */
#define LED_ALL     ((1u<<LED1_PIN)|(1u<<LED2_PIN)|(1u<<LED3_PIN)|(1u<<LED4_PIN))

#define S1_PIN      3u          /* PTB3  */
#define S2_PIN      10u         /* PTB10 */
#define S3_PIN      11u         /* PTB11 */
#define BTN_MASK    ((1u<<S1_PIN)|(1u<<S2_PIN)|(1u<<S3_PIN))

#define PIR_PIN     2u          /* PTC2 = Arduino D6, the shield's D6 header */

#define BUZZ_PIN    1u          /* PTA1, FTM0_CH6 on ALT3 */

#define LATCH_PIN   23u         /* PTB23 */
#define CLK_PIN     3u          /* PTC3  */
#define DATA_PIN    12u         /* PTC12 */

#define LCD_PIN_A   16u         /* PTC16 = D0, APC220 header pin 5 */
#define LCD_PIN_B   17u         /* PTC17 = D1, APC220 header pin 4 */

/*===========================================================================
 *  3. APPLICATION PARAMETERS
 *===========================================================================*/
#define EXIT_DELAY_SEC      10u     /* time to leave after arming      */
#define ENTRY_DELAY_SEC     15u     /* time to disarm after detection  */
#define CODE_LEN            4u
#define DEBOUNCE_RELEASE_MS 30u     /* switch must stay released this long */
#define UNLOCK_SHOW_MS      1500u   /* how long "OPEn" stays on the display */

/* Secret code, expressed as switch numbers: S1, S3, S2, S1 */
static const uint8_t SECRET_CODE[CODE_LEN] = { 1u, 3u, 2u, 1u };

/*===========================================================================
 *  4. SHARED STATE
 *===========================================================================*/
typedef enum {
    ST_DISARMED = 0,
    ST_ARMING,
    ST_ARMED,
    ST_TRIGGERED,
    ST_ALARM,
    ST_UNLOCK                   /* brief "code accepted" confirmation */
} AppState_t;

static volatile uint32_t g_tick_ms    = 0u;   /* milliseconds since reset  */
static volatile uint8_t  g_btn_event  = 0u;   /* 1=S1 2=S2 3=S3, 0=none    */
static volatile uint8_t  g_pir_event  = 0u;   /* set by the PIR interrupt  */
static volatile uint8_t  g_btn_armed  = 1u;   /* debounce: ready for press */
static volatile uint8_t  g_release_ms = 0u;   /* debounce: quiet-time count*/
static uint8_t           g_pir_ready  = 0u;   /* 1 once the PIR line is low */

static volatile uint8_t  g_segbuf[4]  = { 0xFF, 0xFF, 0xFF, 0xFF };

static AppState_t g_state      = ST_DISARMED;
static uint32_t   g_state_ms   = 0u;          /* tick when state was entered */
static uint8_t    g_remain_sec = 0u;          /* countdown shown on display  */
static uint8_t    g_code_idx   = 0u;          /* how many keys entered       */

/*===========================================================================
 *  5. SEVEN-SEGMENT CHARACTER TABLE
 *  Active-low patterns, bit order DP G F E D C B A. A 0 bit lights a segment.
 *===========================================================================*/
static const uint8_t SEG_DIGIT[10] = {
    0xC0, 0xF9, 0xA4, 0xB0, 0x99, 0x92, 0x82, 0xF8, 0x80, 0x90
};

#define SEG_BLANK   0xFFu
#define SEG_DASH    0xBFu
#define SEG_A       0x88u
#define SEG_L       0xC7u
#define SEG_r       0xAFu
#define SEG_n       0xABu
#define SEG_O       0xC0u
#define SEG_P       0x8Cu
#define SEG_E       0x86u

/* Digit-enable masks, left to right. Reorder if the display runs backwards. */
static const uint8_t DIGIT_SELECT[4] = { 0xF1, 0xF2, 0xF4, 0xF8 };

/*===========================================================================
 *  6. LED MODULE                                      (active LOW on PORTD)
 *===========================================================================*/
static const uint8_t LED_MAP[5] = { 0u, LED1_PIN, LED2_PIN, LED3_PIN, LED4_PIN };

static void LED_Init(void)
{
    SIM->SCGC5 |= SIM_SCGC5_PORTD_MASK;
    PORTD->PCR[LED1_PIN] = PORT_PCR_MUX(1);
    PORTD->PCR[LED2_PIN] = PORT_PCR_MUX(1);
    PORTD->PCR[LED3_PIN] = PORT_PCR_MUX(1);
    PORTD->PCR[LED4_PIN] = PORT_PCR_MUX(1);
    PTD->PDDR |= LED_ALL;
    PTD->PSOR  = LED_ALL;                       /* all off */
}

static void LED_On(uint8_t n)
{
    if (n >= 1u && n <= 4u) PTD->PCOR = (1u << LED_MAP[n]);
}

static void LED_Off(uint8_t n)
{
    if (n >= 1u && n <= 4u) PTD->PSOR = (1u << LED_MAP[n]);
}

static void LED_AllOff(void)
{
    PTD->PSOR = LED_ALL;
}

/* Light LED1..LEDn to show how many code keys have been entered. */
static void LED_ShowProgress(uint8_t n)
{
    uint8_t i;
    for (i = 1u; i <= 4u; i++) {
        if (i <= n) LED_On(i); else LED_Off(i);
    }
}

/* Software blink: returns 1 for half of every period_ms window. */
static uint8_t LED_BlinkPhase(uint16_t period_ms)
{
    return (uint8_t)(((g_tick_ms % period_ms) < (period_ms / 2u)) ? 1u : 0u);
}

/* Running light used by the alarm state. */
static void LED_Chase(uint16_t step_ms)
{
    uint8_t pos = (uint8_t)((g_tick_ms / step_ms) & 0x03u);
    LED_AllOff();
    LED_On((uint8_t)(pos + 1u));
}

/*===========================================================================
 *  7. SEVEN-SEGMENT MODULE                            (two cascaded 74HC595)
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

/* Shift one byte into the register chain, most significant bit first. */
static void SevenSeg_ShiftByte(uint8_t b)
{
    int8_t i;
    for (i = 7; i >= 0; i--) {
        if (b & (1u << i)) PTC->PSOR = (1u << DATA_PIN);
        else               PTC->PCOR = (1u << DATA_PIN);
        PTC->PSOR = (1u << CLK_PIN);            /* rising edge shifts a bit */
        PTC->PCOR = (1u << CLK_PIN);
    }
}

/* Drive one digit position with one raw segment pattern.
 * The first byte sent travels furthest and lands in the segment register;
 * the second stays in the nearer register and selects the digit. */
static void SevenSeg_ShowRaw(uint8_t pos, uint8_t pattern)
{
    PTB->PCOR = (1u << LATCH_PIN);              /* hold outputs while shifting */
    SevenSeg_ShiftByte(pattern);
    SevenSeg_ShiftByte(DIGIT_SELECT[pos & 0x03u]);
    PTB->PSOR = (1u << LATCH_PIN);              /* latch both registers at once */
}

static void SevenSeg_SetRaw(uint8_t d0, uint8_t d1, uint8_t d2, uint8_t d3)
{
    g_segbuf[0] = d0;  g_segbuf[1] = d1;
    g_segbuf[2] = d2;  g_segbuf[3] = d3;
}

static void SevenSeg_Blank(void)
{
    SevenSeg_SetRaw(SEG_BLANK, SEG_BLANK, SEG_BLANK, SEG_BLANK);
}

static void SevenSeg_Dashes(void)
{
    SevenSeg_SetRaw(SEG_DASH, SEG_DASH, SEG_DASH, SEG_DASH);
}

/* Right-aligned unsigned number with leading blanks. */
static void SevenSeg_SetNumber(uint16_t v)
{
    uint8_t buf[4];
    int8_t  i;

    if (v > 9999u) v = 9999u;

    for (i = 3; i >= 0; i--) {
        buf[i] = SEG_DIGIT[v % 10u];
        v /= 10u;
    }
    /* blank the leading zeros */
    for (i = 0; i < 3; i++) {
        if (buf[i] == SEG_DIGIT[0]) buf[i] = SEG_BLANK;
        else break;
    }
    SevenSeg_SetRaw(buf[0], buf[1], buf[2], buf[3]);
}

/*===========================================================================
 *  8. BUZZER MODULE                                   (FTM0_CH6 on PTA1)
 *  f_out = BUS_CLK / (prescaler * (MOD + 1)),  prescaler fixed at 64.
 *===========================================================================*/
#define FTM_PRESCALE_DIV    64u
#define TONE_MOD(hz)        ((uint16_t)((BUS_CLK_HZ / FTM_PRESCALE_DIV / (hz)) - 1u))

typedef struct {
    uint16_t freq;      /* Hz, 0 = rest          */
    uint16_t ms;        /* duration, 0 = end     */
} Note_t;

static const Note_t *g_song       = 0;
static uint8_t       g_song_idx   = 0u;
static uint8_t       g_song_loop  = 0u;
static uint32_t      g_note_end   = 0u;

static void Buzzer_Init(void)
{
    SIM->SCGC5 |= SIM_SCGC5_PORTA_MASK;
    SIM->SCGC6 |= SIM_SCGC6_FTM0_MASK;

    FTM0->MODE |= FTM_MODE_WPDIS_MASK;          /* disable write protection */
    FTM0->SC    = 0u;
    FTM0->CNT   = 0u;
    FTM0->CNTIN = 0u;
    FTM0->CONTROLS[6].CnSC = FTM_CnSC_MSB_MASK | FTM_CnSC_ELSB_MASK;

    PORTA->PCR[BUZZ_PIN] = PORT_PCR_MUX(1);
    PTA->PDDR &= ~(1u << BUZZ_PIN);             /* high-Z, silent */
}

/* Silence. The pin becomes an input so no base current can reach the PNP. */
static void Buzzer_NoTone(void)
{
    FTM0->SC = 0u;
    PORTA->PCR[BUZZ_PIN] = PORT_PCR_MUX(1);
    PTA->PDDR &= ~(1u << BUZZ_PIN);
}

static void Buzzer_Tone(uint16_t hz)
{
    uint16_t mod;

    if (hz == 0u) { Buzzer_NoTone(); return; }

    mod = TONE_MOD(hz);
    FTM0->SC  = 0u;
    FTM0->CNT = 0u;
    FTM0->MOD = mod;
    FTM0->CONTROLS[6].CnV = (uint16_t)(mod / 2u);       /* 50 % duty */
    PORTA->PCR[BUZZ_PIN] = PORT_PCR_MUX(3);             /* route to FTM0_CH6 */
    FTM0->SC = FTM_SC_CLKS(1) | FTM_SC_PS(6);           /* system clock / 64 */
}

static void Buzzer_Stop(void)
{
    g_song = 0;
    Buzzer_NoTone();
}

/* Begin the note the index currently points at. */
static void Buzzer_StartNote(void)
{
    if (g_song == 0) return;

    if (g_song[g_song_idx].ms == 0u) {          /* end-of-song marker */
        if (g_song_loop) {
            g_song_idx = 0u;
        } else {
            Buzzer_Stop();
            return;
        }
    }
    Buzzer_Tone(g_song[g_song_idx].freq);
    g_note_end = g_tick_ms + g_song[g_song_idx].ms;
}

static void Buzzer_Play(const Note_t *song, uint8_t loop)
{
    g_song      = song;
    g_song_idx  = 0u;
    g_song_loop = loop;
    Buzzer_StartNote();
}

/* Must be called often from the main loop. Never blocks. */
static void Buzzer_Task(void)
{
    if (g_song == 0) return;
    if ((int32_t)(g_tick_ms - g_note_end) < 0) return;

    g_song_idx++;
    Buzzer_StartNote();
}

/*--- Melody definitions ---------------------------------------------------*/
static const Note_t SONG_BEEP[]   = { {1200u, 60u}, {0u, 0u} };
static const Note_t SONG_BEEP_HI[]= { {2000u, 70u}, {0u, 0u} };
static const Note_t SONG_ARMED[]  = { { 784u,100u}, {1047u,160u}, {0u, 0u} };
static const Note_t SONG_OK[]     = { { 523u,110u}, { 659u,110u}, {784u,220u}, {0u,0u} };
static const Note_t SONG_ERROR[]  = { { 220u,180u}, {   0u, 60u}, {175u,260u}, {0u,0u} };
static const Note_t SONG_SIREN[]  = { { 880u,180u}, { 660u,180u}, {0u, 0u} };   /* looped */

/*===========================================================================
 *  9. SWITCH MODULE                                   (PORTB, active LOW)
 *===========================================================================*/
static void Button_Init(void)
{
    SIM->SCGC5 |= SIM_SCGC5_PORTB_MASK;

    /* GPIO, pull-up enabled, interrupt on falling edge (IRQC = 1010b) */
    PORTB->PCR[S1_PIN] = PORT_PCR_MUX(1) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK
                       | PORT_PCR_IRQC(0x0A);
    PORTB->PCR[S2_PIN] = PORT_PCR_MUX(1) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK
                       | PORT_PCR_IRQC(0x0A);
    PORTB->PCR[S3_PIN] = PORT_PCR_MUX(1) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK
                       | PORT_PCR_IRQC(0x0A);
    PTB->PDDR &= ~BTN_MASK;

    PORTB->ISFR = 0xFFFFFFFFu;
    NVIC_SetPriority(PORTB_IRQn, 3);
    NVIC_EnableIRQ(PORTB_IRQn);
}

/* Read and clear the pending switch event in one step. */
static uint8_t Button_GetEvent(void)
{
    uint8_t ev = g_btn_event;
    if (ev != 0u) g_btn_event = 0u;
    return ev;
}

/*===========================================================================
 * 10. PIR MODULE                                      (HC-SR501 on PTC2)
 *===========================================================================*/
static void PIR_Init(void)
{
#if (PIR_USE_HARDWARE == 1)
    SIM->SCGC5 |= SIM_SCGC5_PORTC_MASK;
    /* GPIO input, interrupt on rising edge (IRQC = 1001b). The module drives
     * the line actively, so no pull resistor is enabled here. */
    PORTC->PCR[PIR_PIN] = PORT_PCR_MUX(1) | PORT_PCR_IRQC(0x09);
    PTC->PDDR &= ~(1u << PIR_PIN);
    PORTC->ISFR = (1u << PIR_PIN);
    NVIC_SetPriority(PORTC_IRQn, 3);
    NVIC_EnableIRQ(PORTC_IRQn);
#endif
}

static uint8_t PIR_GetEvent(void)
{
    uint8_t ev = g_pir_event;
    if (ev != 0u) g_pir_event = 0u;
    return ev;
}

/*===========================================================================
 * 11. LCD MODULE              (16x2 HD44780 LCD behind a PCF8574 I2C backpack)
 *  The I2C bus is driven in software on PTC16/PTC17 (hardware note 5).
 *  Open-drain emulation:  LOW  = pin is an output driving 0
 *                         HIGH = pin is an input, the backpack pull-up
 *                                raises the line
 *  PCF8574 port bits: P0 = RS, P1 = RW, P2 = EN, P3 = backlight,
 *                     P4..P7 = LCD D4..D7 (the LCD runs in 4-bit mode).
 *  With USE_LCD = 0 every call is a no-op.
 *===========================================================================*/
#if (USE_LCD == 1)

#define LCD_RS          0x01u
#define LCD_EN          0x04u
#define LCD_BL          0x08u
#define LCD_COLS        16u
#define SWI2C_HALF_US   10u         /* half a clock period, bus stays < 50 kHz */

static uint32_t g_i2c_sda  = (1u << LCD_PIN_A);
static uint32_t g_i2c_scl  = (1u << LCD_PIN_B);
static uint8_t  g_lcd_addr = 0u;    /* 7-bit address, 0 = no LCD answered */

/* Busy wait of at least 'us' microseconds. One pass of the loop takes at
 * least 3 core cycles, and the core runs at the bus clock in CLOCK_SETUP 0,
 * so the real delay is never shorter than asked (usually 1.5-2x longer). */
static void LCD_DelayUs(uint32_t us)
{
    volatile uint32_t n = us * ((BUS_CLK_HZ / 3000000u) + 1u);
    while (n != 0u) { n--; }
}

static void LCD_DelayMs(uint32_t ms)
{
    while (ms != 0u) {
        LCD_DelayUs(1000u);
        ms--;
    }
}

/*--- bit-banged I2C master ------------------------------------------------*/
static void SWI2C_SdaLow(void)  { PTC->PDDR |=  g_i2c_sda; }
static void SWI2C_SdaHigh(void) { PTC->PDDR &= ~g_i2c_sda; }
static void SWI2C_SclLow(void)  { PTC->PDDR |=  g_i2c_scl; }

/* Release SCL, then wait (with a time-out) until the line is really high. */
static void SWI2C_SclHigh(void)
{
    uint32_t guard = 1000u;

    PTC->PDDR &= ~g_i2c_scl;
    while (((PTC->PDIR & g_i2c_scl) == 0u) && (guard != 0u)) {
        guard--;
    }
}

/* START: SDA falls while SCL is high. */
static void SWI2C_Start(void)
{
    SWI2C_SdaHigh();
    SWI2C_SclHigh();    LCD_DelayUs(SWI2C_HALF_US);
    SWI2C_SdaLow();     LCD_DelayUs(SWI2C_HALF_US);
    SWI2C_SclLow();     LCD_DelayUs(SWI2C_HALF_US);
}

/* STOP: SDA rises while SCL is high. Expects SCL low on entry. */
static void SWI2C_Stop(void)
{
    SWI2C_SdaLow();     LCD_DelayUs(SWI2C_HALF_US);
    SWI2C_SclHigh();    LCD_DelayUs(SWI2C_HALF_US);
    SWI2C_SdaHigh();    LCD_DelayUs(SWI2C_HALF_US);
}

/* Send one byte, MSB first. Returns 1 if the slave answered with ACK. */
static uint8_t SWI2C_WriteByte(uint8_t b)
{
    uint8_t i;
    uint8_t ack;

    for (i = 0u; i < 8u; i++) {
        if (b & 0x80u) SWI2C_SdaHigh(); else SWI2C_SdaLow();
        b = (uint8_t)(b << 1);
        LCD_DelayUs(SWI2C_HALF_US);
        SWI2C_SclHigh();    LCD_DelayUs(SWI2C_HALF_US);
        SWI2C_SclLow();
    }

    SWI2C_SdaHigh();                            /* let the slave drive SDA */
    LCD_DelayUs(SWI2C_HALF_US);
    SWI2C_SclHigh();    LCD_DelayUs(SWI2C_HALF_US);
    ack = (uint8_t)(((PTC->PDIR & g_i2c_sda) == 0u) ? 1u : 0u);
    SWI2C_SclLow();     LCD_DelayUs(SWI2C_HALF_US);
    return ack;
}

/* Free a slave left halfway through a byte: 9 clocks with SDA released,
 * then a STOP. */
static void SWI2C_Recover(void)
{
    uint8_t i;

    SWI2C_SdaHigh();
    for (i = 0u; i < 9u; i++) {
        SWI2C_SclLow();     LCD_DelayUs(SWI2C_HALF_US);
        SWI2C_SclHigh();    LCD_DelayUs(SWI2C_HALF_US);
    }
    SWI2C_SclLow();     LCD_DelayUs(SWI2C_HALF_US);
    SWI2C_Stop();
}

/* Returns 1 if a device answers at this 7-bit address. */
static uint8_t SWI2C_Probe(uint8_t addr)
{
    uint8_t ack;

    SWI2C_Start();
    ack = SWI2C_WriteByte((uint8_t)(addr << 1));    /* R/W bit = 0, write */
    SWI2C_Stop();
    return ack;
}

/*--- PCF8574 -> HD44780 ---------------------------------------------------*/

/* Put one nibble on D4..D7 and pulse EN. Must be called inside an open I2C
 * transfer. Each PCF8574 byte takes well over 0.1 ms on this slow bus, so
 * the EN pulse width and the 37 us HD44780 execution time are both met. */
static void LCD_PutNibble(uint8_t nibble, uint8_t rs)
{
    uint8_t out = (uint8_t)((uint8_t)(nibble << 4) | LCD_BL | rs);

    (void)SWI2C_WriteByte((uint8_t)(out | LCD_EN));
    (void)SWI2C_WriteByte(out);                 /* EN falls: LCD latches */
}

static void LCD_PutByte(uint8_t b, uint8_t rs)
{
    LCD_PutNibble((uint8_t)(b >> 4), rs);
    LCD_PutNibble((uint8_t)(b & 0x0Fu), rs);
}

/* Start-up only: a single nibble as its own I2C transfer. */
static void LCD_InitNibble(uint8_t nibble)
{
    SWI2C_Start();
    (void)SWI2C_WriteByte((uint8_t)(g_lcd_addr << 1));
    LCD_PutNibble(nibble, 0u);
    SWI2C_Stop();
}

/* One command byte as its own I2C transfer. */
static void LCD_Command(uint8_t cmd)
{
    SWI2C_Start();
    (void)SWI2C_WriteByte((uint8_t)(g_lcd_addr << 1));
    LCD_PutByte(cmd, 0u);
    SWI2C_Stop();
}

/* Try both SDA/SCL orders and every PCF8574 (0x20-0x27) and PCF8574A
 * (0x38-0x3F) address. Returns the address that answered and leaves the
 * working pin order selected, or returns 0 if nothing answered. */
static uint8_t LCD_Detect(void)
{
    uint8_t order;
    uint8_t addr;

    for (order = 0u; order < 2u; order++) {
        g_i2c_sda = (order == 0u) ? (1u << LCD_PIN_A) : (1u << LCD_PIN_B);
        g_i2c_scl = (order == 0u) ? (1u << LCD_PIN_B) : (1u << LCD_PIN_A);
        SWI2C_Recover();

        for (addr = 0x20u; addr <= 0x3Fu; addr++) {
            if (addr == 0x28u) addr = 0x38u;    /* skip the unused gap */
            if (SWI2C_Probe(addr)) return addr;
        }
    }
    return 0u;
}

/* Rewrite one whole row, padded with spaces, in a single I2C transfer. */
static void LCD_WriteLine(uint8_t ddram_cmd, const char *s)
{
    uint8_t col;

    SWI2C_Start();
    (void)SWI2C_WriteByte((uint8_t)(g_lcd_addr << 1));
    LCD_PutByte(ddram_cmd, 0u);                 /* cursor to start of row */
    for (col = 0u; col < LCD_COLS; col++) {
        if (*s != '\0') {
            LCD_PutByte((uint8_t)*s, LCD_RS);
            s++;
        } else {
            LCD_PutByte((uint8_t)' ', LCD_RS);
        }
    }
    SWI2C_Stop();
}

#endif /* USE_LCD */

static void LCD_Init(void)
{
#if (USE_LCD == 1)
    SIM->SCGC5 |= SIM_SCGC5_PORTC_MASK;
    PORTC->PCR[LCD_PIN_A] = PORT_PCR_MUX(1);    /* GPIO, no internal pull */
    PORTC->PCR[LCD_PIN_B] = PORT_PCR_MUX(1);
    PTC->PCOR  =  (1u << LCD_PIN_A) | (1u << LCD_PIN_B);  /* output level 0 */
    PTC->PDDR &= ~((1u << LCD_PIN_A) | (1u << LCD_PIN_B)); /* both released */

    LCD_DelayMs(50u);                           /* LCD power-up time */
    g_lcd_addr = LCD_Detect();
    if (g_lcd_addr == 0u) return;               /* no LCD: stay a no-op */

    /* HD44780 reset sequence, ending in 4-bit mode */
    LCD_InitNibble(0x3u);   LCD_DelayMs(5u);
    LCD_InitNibble(0x3u);   LCD_DelayMs(5u);
    LCD_InitNibble(0x3u);   LCD_DelayMs(1u);
    LCD_InitNibble(0x2u);   LCD_DelayMs(1u);

    LCD_Command(0x28u);     /* 4-bit bus, 2 rows, 5x8 font */
    LCD_Command(0x0Cu);     /* display on, cursor off      */
    LCD_Command(0x06u);     /* cursor moves right          */
    LCD_Command(0x01u);     /* clear                       */
    LCD_DelayMs(5u);
#endif
}

/* Show two rows of up to 16 characters. Blocks for roughly 0.1 s. */
static void LCD_Show(const char *line1, const char *line2)
{
#if (USE_LCD == 1)
    if (g_lcd_addr == 0u) return;
    LCD_WriteLine(0x80u, line1);                /* row 0 = DDRAM 0x00 */
    LCD_WriteLine(0xC0u, line2);                /* row 1 = DDRAM 0x40 */
#else
    (void)line1; (void)line2;
#endif
}

/* Show the LCD_Detect() result on the 7-segment display for 1.5 s:
 * "L 27" / "L 3F" = address that answered, "L --" = no LCD found.
 * Needs the PIT tick, so main() calls it after System_Init(). */
static void LCD_BootReport(void)
{
#if (USE_LCD == 1)
    static const uint8_t SEG_HEX[16] = {
        0xC0, 0xF9, 0xA4, 0xB0, 0x99, 0x92, 0x82, 0xF8,
        0x80, 0x90, 0x88, 0x83, 0xC6, 0xA1, 0x86, 0x8E
    };
    uint32_t t0;

    if (g_lcd_addr != 0u) {
        SevenSeg_SetRaw(SEG_L, SEG_BLANK,
                        SEG_HEX[g_lcd_addr >> 4], SEG_HEX[g_lcd_addr & 0x0Fu]);
    } else {
        SevenSeg_SetRaw(SEG_L, SEG_BLANK, SEG_DASH, SEG_DASH);
    }

    t0 = g_tick_ms;
    while ((g_tick_ms - t0) < 1500u) { }
#endif
}

/*===========================================================================
 * 12. TIMER MODULE                                    (PIT channels 0 and 1)
 *  LDVAL = (period * bus clock) - 1
 *===========================================================================*/
#define PIT_LDVAL_MS(ms)    (((BUS_CLK_HZ / 1000u) * (ms)) - 1u)

static void PIT_Init(void)
{
    SIM->SCGC6 |= SIM_SCGC6_PIT_MASK;
    PIT->MCR = 0u;                              /* module on, runs in debug */

    PIT->CHANNEL[0].LDVAL = PIT_LDVAL_MS(2);    /* display refresh */
    PIT->CHANNEL[0].TCTRL = PIT_TCTRL_TIE_MASK | PIT_TCTRL_TEN_MASK;

    PIT->CHANNEL[1].LDVAL = PIT_LDVAL_MS(1);    /* system tick */
    PIT->CHANNEL[1].TCTRL = PIT_TCTRL_TIE_MASK | PIT_TCTRL_TEN_MASK;

    NVIC_SetPriority(PIT0_IRQn, 1);             /* display must not stutter */
    NVIC_SetPriority(PIT1_IRQn, 2);
    NVIC_EnableIRQ(PIT0_IRQn);
    NVIC_EnableIRQ(PIT1_IRQn);
}

/*===========================================================================
 * 13. INTERRUPT SERVICE ROUTINES
 *  Every handler clears its flag first and then issues __DSB(). Without the
 *  barrier the clearing write can still sit in the write buffer when the
 *  handler returns, so the core may immediately re-enter the same ISR.
 *  Handlers stay short: they only update flags and counters.
 *===========================================================================*/

/* 2 ms : advance the display multiplex by one digit. */
void PIT0_IRQHandler(void)
{
    static uint8_t pos = 0u;

    PIT->CHANNEL[0].TFLG = PIT_TFLG_TIF_MASK;
    __DSB();

    SevenSeg_ShowRaw(pos, g_segbuf[pos]);
    pos = (uint8_t)((pos + 1u) & 0x03u);
}

/* 1 ms : system tick, and the release half of the switch debounce. */
void PIT1_IRQHandler(void)
{
    PIT->CHANNEL[1].TFLG = PIT_TFLG_TIF_MASK;
    __DSB();

    g_tick_ms++;

    /* A new press is only accepted after every switch has been released
     * continuously for DEBOUNCE_RELEASE_MS. This rejects both the press
     * bounce and the release bounce of the contacts. */
    if ((PTB->PDIR & BTN_MASK) == BTN_MASK) {
        if (g_release_ms < DEBOUNCE_RELEASE_MS) {
            g_release_ms++;
            if (g_release_ms >= DEBOUNCE_RELEASE_MS) g_btn_armed = 1u;
        }
    } else {
        g_release_ms = 0u;
    }
}

/* Switches S1, S2 and S3 share the PORTB vector. The source is identified
 * from the interrupt status flag register. */
void PORTB_IRQHandler(void)
{
    uint32_t flags = PORTB->ISFR;
    uint32_t pdir;

    PORTB->ISFR = flags;                        /* write-1-to-clear */
    __DSB();

    if (!g_btn_armed) return;                   /* previous press not released */

    pdir = PTB->PDIR;                           /* confirm the line really is low */

    if      ((flags & (1u << S1_PIN)) && !(pdir & (1u << S1_PIN))) g_btn_event = 1u;
    else if ((flags & (1u << S2_PIN)) && !(pdir & (1u << S2_PIN))) g_btn_event = 2u;
    else if ((flags & (1u << S3_PIN)) && !(pdir & (1u << S3_PIN))) g_btn_event = 3u;
    else return;                                /* glitch, ignore */

    g_btn_armed = 0u;
}

#if (PIR_USE_HARDWARE == 1)
/* The PIR sits on PTC2, so it has its own port vector. PTC3, PTC12, PTC16
 * and PTC17 belong to the 7-segment and LCD drivers but keep IRQC = 0, so
 * they never reach this handler. The flag is cleared first, then PDIR
 * confirms the line really is high before a motion event is recorded. */
void PORTC_IRQHandler(void)
{
    uint32_t flags = PORTC->ISFR;

    PORTC->ISFR = flags;                        /* write-1-to-clear */
    __DSB();

    if ((flags & (1u << PIR_PIN)) && (PTC->PDIR & (1u << PIR_PIN))) {
        g_pir_event = 1u;
    }
}
#endif

/* Visible crash indicator: all four LEDs on solid. */
void HardFault_Handler(void)
{
    SIM->SCGC5 |= SIM_SCGC5_PORTD_MASK;
    PORTD->PCR[LED1_PIN] = PORT_PCR_MUX(1);
    PORTD->PCR[LED2_PIN] = PORT_PCR_MUX(1);
    PORTD->PCR[LED3_PIN] = PORT_PCR_MUX(1);
    PORTD->PCR[LED4_PIN] = PORT_PCR_MUX(1);
    PTD->PDDR |= LED_ALL;
    PTD->PCOR  = LED_ALL;
    for (;;) { }
}

/*===========================================================================
 * 14. SECRET CODE MODULE
 *===========================================================================*/
static void Code_Reset(void)
{
    g_code_idx = 0u;
}

/* Returns 0 = still collecting, 1 = complete and correct, 2 = complete and wrong */
static uint8_t Code_AddKey(uint8_t key)
{
    if (key != SECRET_CODE[g_code_idx]) {       /* wrong key at this position */
        g_code_idx = 0u;
        return 2u;
    }

    g_code_idx++;
    if (g_code_idx >= CODE_LEN) {
        g_code_idx = 0u;
        return 1u;
    }
    return 0u;
}

/*===========================================================================
 * 15. STATE MACHINE
 *===========================================================================*/
static void App_EnterState(AppState_t s)
{
    g_state    = s;
    g_state_ms = g_tick_ms;
    Code_Reset();
    LED_AllOff();
    Buzzer_Stop();
    g_pir_event = 0u;        /* drop anything the sensor reported earlier */

    switch (s) {
    case ST_DISARMED:
        g_remain_sec = 0u;
        SevenSeg_Dashes();
        LCD_Show("SYSTEM DISARMED", "S1 = arm");
        break;

    case ST_ARMING:
        g_remain_sec = EXIT_DELAY_SEC;
        SevenSeg_SetNumber(g_remain_sec);
        LCD_Show("ARMING...", "please leave");
        break;

    case ST_ARMED:
        g_remain_sec = 0u;
        g_pir_ready  = 0u;   /* trust the PIR only after its line settles low */
        SevenSeg_SetRaw(SEG_BLANK, SEG_O, SEG_n, SEG_BLANK);
        LCD_Show("ARMED - SECURE", "");
        Buzzer_Play(SONG_ARMED, 0u);
        break;

    case ST_TRIGGERED:
        g_remain_sec = ENTRY_DELAY_SEC;
        SevenSeg_SetNumber(g_remain_sec);
        LCD_Show("INTRUDER!", "enter code");
        break;

    case ST_ALARM:
        g_remain_sec = 0u;
        LCD_Show("*** ALARM ***", "enter code");
        Buzzer_Play(SONG_SIREN, 1u);            /* looping siren */
        break;

    case ST_UNLOCK:
        g_remain_sec = 0u;
        SevenSeg_SetRaw(SEG_O, SEG_P, SEG_E, SEG_n);
        LCD_Show("CODE ACCEPTED", "welcome home");
        Buzzer_Play(SONG_OK, 0u);
        LED_On(1); LED_On(2); LED_On(3); LED_On(4);
        break;

    default:
        break;
    }
}

/* Shared by TRIGGERED and ALARM: collect code keys and act on the result. */
static uint8_t App_HandleCodeEntry(uint8_t key)
{
    uint8_t result;

    if (key == 0u) return 0u;

    result = Code_AddKey(key);

    if (result == 1u) {                         /* correct, caller transitions */
        return 1u;
    }
    if (result == 2u) {                         /* wrong */
        Buzzer_Play(SONG_ERROR, 0u);
        return 0u;
    }
    Buzzer_Play(SONG_BEEP, 0u);                 /* accepted one key */
    return 0u;
}

static void State_Disarmed(void)
{
    uint8_t key = Button_GetEvent();

    LED_On(1);                                  /* steady: system is off */

    if (key == 1u) {
        App_EnterState(ST_ARMING);
    }
}

static void State_Arming(void)
{
    uint8_t  key     = Button_GetEvent();
    uint32_t elapsed = g_tick_ms - g_state_ms;
    uint8_t  left;

    if (key != 0u) {                            /* any key cancels */
        App_EnterState(ST_DISARMED);
        return;
    }

    if (LED_BlinkPhase(500u)) LED_On(2); else LED_Off(2);   /* 2 Hz */

    left = (uint8_t)(EXIT_DELAY_SEC - (elapsed / 1000u));
    if (elapsed >= (uint32_t)EXIT_DELAY_SEC * 1000u) {
        App_EnterState(ST_ARMED);
        return;
    }

    if (left != g_remain_sec) {                 /* one second has passed */
        g_remain_sec = left;
        SevenSeg_SetNumber(g_remain_sec);
        Buzzer_Play(SONG_BEEP, 0u);
    }
}

static void State_Armed(void)
{
    uint8_t key = Button_GetEvent();

    if (LED_BlinkPhase(2000u)) LED_On(2); else LED_Off(2);

#if (PIR_USE_HARDWARE == 1)
    /* LED4 mirrors the raw PIR output so the sensor can be watched live. */
    if (PTC->PDIR & (1u << PIR_PIN)) LED_On(4); else LED_Off(4);

    /* After the exit walk the PIR usually still holds its output high, and
     * it keeps it there for as long as its Tx pot is set to. A rising edge
     * can only arrive once the line has fallen again, so the sensor is
     * ignored until it has been seen low at least once. LED3 is on while
     * the system is still waiting for that. */
    if (!g_pir_ready) {
        LED_On(3);
        g_pir_event = 0u;
        if ((PTC->PDIR & (1u << PIR_PIN)) == 0u) g_pir_ready = 1u;
    } else {
        LED_Off(3);
        if (PIR_GetEvent()) {
            App_EnterState(ST_TRIGGERED);
            return;
        }
    }
#else
    if (key == 3u) g_pir_event = 1u;            /* S3 stands in for the PIR */

    if (PIR_GetEvent()) {
        App_EnterState(ST_TRIGGERED);
        return;
    }
#endif

    if (key == 1u) {                            /* owner coming home */
        App_EnterState(ST_TRIGGERED);
    }
}

static void State_Triggered(void)
{
    uint8_t  key     = Button_GetEvent();
    uint32_t elapsed = g_tick_ms - g_state_ms;
    uint8_t  left;

    LED_ShowProgress(g_code_idx);

    if (App_HandleCodeEntry(key)) {
        App_EnterState(ST_UNLOCK);
        return;
    }

    if (elapsed >= (uint32_t)ENTRY_DELAY_SEC * 1000u) {
        App_EnterState(ST_ALARM);
        return;
    }

    left = (uint8_t)(ENTRY_DELAY_SEC - (elapsed / 1000u));
    if (left != g_remain_sec) {
        g_remain_sec = left;
        SevenSeg_SetNumber(g_remain_sec);
        /* the warning beep gets higher as the remaining time shrinks */
        if (g_song == 0) {
            Buzzer_Play((left <= 5u) ? SONG_BEEP_HI : SONG_BEEP, 0u);
        }
    }
}

static void State_Alarm(void)
{
    uint8_t key = Button_GetEvent();

    LED_Chase(120u);

    /* blink "ALAr" on and off twice a second */
    if (LED_BlinkPhase(1000u)) {
        SevenSeg_SetRaw(SEG_A, SEG_L, SEG_A, SEG_r);
    } else {
        SevenSeg_Blank();
    }

    if (App_HandleCodeEntry(key)) {
        App_EnterState(ST_UNLOCK);
    }
}

/* Holds the "OPEn" confirmation long enough to be seen and heard. */
static void State_Unlock(void)
{
    (void)Button_GetEvent();                    /* swallow stray presses */

    if ((g_tick_ms - g_state_ms) >= UNLOCK_SHOW_MS) {
        App_EnterState(ST_DISARMED);
    }
}

static void App_Run(void)
{
    switch (g_state) {
    case ST_DISARMED:  State_Disarmed();  break;
    case ST_ARMING:    State_Arming();    break;
    case ST_ARMED:     State_Armed();     break;
    case ST_TRIGGERED: State_Triggered(); break;
    case ST_ALARM:     State_Alarm();     break;
    case ST_UNLOCK:    State_Unlock();    break;
    default:           App_EnterState(ST_DISARMED); break;
    }
}

/*===========================================================================
 * 16. MAIN
 *===========================================================================*/
static void System_Init(void)
{
    LED_Init();
    SevenSeg_Init();
    Buzzer_Init();
    Button_Init();
    PIR_Init();
    LCD_Init();
    PIT_Init();                                 /* start the timers last */
}

int main(void)
{
    System_Init();
    LCD_BootReport();       /* 1.5 s: shows whether the LCD answered */
    App_EnterState(ST_DISARMED);

    for (;;) {
        Buzzer_Task();      /* advance the melody, never blocks */
        App_Run();          /* run the current state handler    */
    }
}
