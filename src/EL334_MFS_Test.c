/*=============================================================================
 *  EL334 Mini-Project (1/69) - Hardware Bring-up Test Program
 *  Platform : NXP FRDM-K64F + Arduino Multi-Function Shield (MFS)
 *  Purpose  : ทดสอบอุปกรณ์บนชีลด์ทีละส่วน ก่อนประกอบเป็น State Machine จริง
 *
 *  วิธีใช้ : แก้ค่า TEST_STAGE ด้านล่างเป็น 1..7 แล้ว Build + Debug ทีละสเตจ
 *
 *---------------------------------------------------------------------------
 *  PIN MAP (ตรวจสอบจาก Schematic ของ MFS + FRDM-K64F Arduino header)
 *---------------------------------------------------------------------------
 *  อุปกรณ์            Arduino   K64F      หมายเหตุ
 *  LED1               D13       PTD1      Active LOW (anode ไป +5V ผ่าน R)
 *  LED2               D12       PTD3      Active LOW
 *  LED3               D11       PTD2      Active LOW
 *  LED4               D10       PTD0      Active LOW
 *  S1                 A1        PTB3      Active LOW, ใช้ internal pull-up
 *  S2                 A2        PTB10     Active LOW
 *  S3                 A3        PTB11     Active LOW
 *  Buzzer             D3        PTA1      ผ่าน PNP 2N3906 -> Active LOW
 *  7-Seg LATCH        D4        PTB23     74HC595 LTCHCLK
 *  7-Seg CLK          D7        PTC3      74HC595 SFTCLK
 *  7-Seg DATA         D8        PTC12     74HC595 SDI
 *  Potentiometer      A0        PTB2      *** อ่าน WARNING #3 ***
 *
 *---------------------------------------------------------------------------
 *  !!! HARDWARE WARNING - อ่านก่อนเสียบบอร์ด !!!
 *---------------------------------------------------------------------------
 *  #1  ปุ่ม S1-S3 : บนชีลด์มี R6/R7/R8 = 10K pull-up ต่อไป +5V ผ่านจัมเปอร์ J2
 *      K64F เป็น 3.3V และขา PTB3/PTB10/PTB11 ไม่ทน 5V
 *      >>> ให้ถอดจัมเปอร์ J2 ออก แล้วใช้ internal pull-up ของ K64F แทน
 *          (โค้ดนี้เปิด PE=1, PS=1 ไว้ให้แล้ว)
 *
 *  #2  Buzzer : ขับด้วย PNP 2N3906 (emitter ไป +5V, base ผ่าน R9 10K มาที่ PTA1)
 *      ถ้าสั่งขา HIGH ที่ 3.3V -> Veb = 5.0 - 3.3 = 1.7V ซึ่งยังมากกว่า 0.7V
 *      ทรานซิสเตอร์จะยังนำกระแสอยู่ => buzzer ไม่ดับสนิท (มีเสียงหวี่ๆ)
 *      >>> วิธีปิดที่ถูกต้องคือตั้งขาเป็น INPUT (high-Z) ให้ Ib = 0
 *          ดูฟังก์ชัน Buzzer_Off()
 *
 *  #3  Potentiometer : R10 ต่อไป +5V ทำให้ขา A0 แกว่งได้ถึง 5V
 *      PTB2 ไม่ทน 5V >>> อย่าใช้ ADC กับ A0 ถ้ายังไม่ได้ใส่ voltage divider
 *      (แก้คำแนะนำเดิม: ไม่ควรใช้ pot ตรงๆ)
 *      เช่นเดียวกับซ็อกเก็ต DS18B20 ที่มี R5 10K pull-up ไป +5V ผ่าน J1
 *
 *===========================================================================*/

#include "MK64F12.h"          /* ถ้า error ให้เปลี่ยนเป็น "fsl_device_registers.h" */
#include <stdint.h>

/*===========================================================================
 *  เลือกสเตจที่จะทดสอบ : 1..7
 *===========================================================================*/
#define TEST_STAGE   1

/*---------------------------------------------------------------------------
 *  Clock - ค่าเริ่มต้นของ SDK คือ core 120 MHz / bus 60 MHz
 *  ถ้าโปรเจคไม่ได้เรียก BOARD_InitBootClocks() ค่าจะเป็น ~20.97 MHz
 *  ตรวจสอบด้วยการ watch ตัวแปร SystemCoreClock ตอน debug
 *-------------------------------------------------------------------------*/
#define BUS_CLK_HZ          60000000u
#define DELAY_LOOPS_PER_MS  12000u      /* ปรับจูนเองได้ ใช้แค่ในการทดสอบ */

/*--------------------------- Pin definitions -------------------------------*/
#define LED1_PIN    1u      /* PTD1 */
#define LED2_PIN    3u      /* PTD3 */
#define LED3_PIN    2u      /* PTD2 */
#define LED4_PIN    0u      /* PTD0 */
#define LED_ALL     ((1u<<LED1_PIN)|(1u<<LED2_PIN)|(1u<<LED3_PIN)|(1u<<LED4_PIN))

#define S1_PIN      3u      /* PTB3  */
#define S2_PIN      10u     /* PTB10 */
#define S3_PIN      11u     /* PTB11 */

#define BUZZ_PIN    1u      /* PTA1  */

#define LATCH_PIN   23u     /* PTB23 */
#define CLK_PIN     3u      /* PTC3  */
#define DATA_PIN    12u     /* PTC12 */

/*--------------------------- 7-Segment tables ------------------------------*/
/* Active-low segment patterns, bit order: DP G F E D C B A (0 = ติด) */
static const uint8_t SEGMENTS[10] = {
    0xC0, 0xF9, 0xA4, 0xB0, 0x99, 0x92, 0x82, 0xF8, 0x80, 0x90
};
/* ตัวเลือกหลัก ซ้าย -> ขวา (ถ้าหลักสลับกัน ให้สลับลำดับในอาเรย์นี้) */
static const uint8_t DIGIT_SELECT[4] = { 0xF1, 0xF2, 0xF4, 0xF8 };

static volatile uint8_t  g_disp[4] = {0, 0, 0, 0};   /* บัฟเฟอร์แสดงผล */
static volatile uint32_t g_tick_ms = 0;              /* นับเวลาจาก PIT1 */

/*===========================================================================
 *  Low-level delay (ใช้เฉพาะตอนทดสอบ ไม่ใช้ในโปรเจคจริง)
 *===========================================================================*/
static void delay_ms(uint32_t ms)
{
    volatile uint32_t i;
    while (ms--) {
        for (i = 0; i < DELAY_LOOPS_PER_MS; i++) { __NOP(); }
    }
}

/*===========================================================================
 *  LED  (active LOW)
 *===========================================================================*/
static void LED_Init(void)
{
    SIM->SCGC5 |= SIM_SCGC5_PORTD_MASK;

    PORTD->PCR[LED1_PIN] = PORT_PCR_MUX(1);
    PORTD->PCR[LED2_PIN] = PORT_PCR_MUX(1);
    PORTD->PCR[LED3_PIN] = PORT_PCR_MUX(1);
    PORTD->PCR[LED4_PIN] = PORT_PCR_MUX(1);

    PTD->PDDR |= LED_ALL;     /* output */
    PTD->PSOR  = LED_ALL;     /* HIGH = ดับ (active low) */
}

static void LED_On(uint8_t n)      /* n = 1..4 */
{
    static const uint8_t map[5] = {0, LED1_PIN, LED2_PIN, LED3_PIN, LED4_PIN};
    if (n >= 1 && n <= 4) PTD->PCOR = (1u << map[n]);
}

static void LED_Off(uint8_t n)
{
    static const uint8_t map[5] = {0, LED1_PIN, LED2_PIN, LED3_PIN, LED4_PIN};
    if (n >= 1 && n <= 4) PTD->PSOR = (1u << map[n]);
}

static void LED_Toggle(uint8_t n)
{
    static const uint8_t map[5] = {0, LED1_PIN, LED2_PIN, LED3_PIN, LED4_PIN};
    if (n >= 1 && n <= 4) PTD->PTOR = (1u << map[n]);
}

static void LED_AllOff(void) { PTD->PSOR = LED_ALL; }

/*===========================================================================
 *  Buttons  (active LOW + internal pull-up)
 *===========================================================================*/
static void Button_Init(void)
{
    SIM->SCGC5 |= SIM_SCGC5_PORTB_MASK;

    PORTB->PCR[S1_PIN] = PORT_PCR_MUX(1) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK;
    PORTB->PCR[S2_PIN] = PORT_PCR_MUX(1) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK;
    PORTB->PCR[S3_PIN] = PORT_PCR_MUX(1) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK;

    PTB->PDDR &= ~((1u<<S1_PIN) | (1u<<S2_PIN) | (1u<<S3_PIN));   /* input */
}

/* คืนค่า 1 เมื่อกดอยู่ (ขาเป็น LOW) */
static uint8_t Button_IsPressed(uint8_t pin)
{
    return ((PTB->PDIR & (1u << pin)) == 0u) ? 1u : 0u;
}

/*===========================================================================
 *  Buzzer - โหมด GPIO (เปิด/ปิดเฉยๆ)
 *  ON  = ขา output LOW   (PNP นำกระแส)
 *  OFF = ขาเป็น input    (ตัดกระแส base สนิท - ดู WARNING #2)
 *===========================================================================*/
static void Buzzer_InitGPIO(void)
{
    SIM->SCGC5 |= SIM_SCGC5_PORTA_MASK;
    PORTA->PCR[BUZZ_PIN] = PORT_PCR_MUX(1);
    PTA->PDDR &= ~(1u << BUZZ_PIN);         /* เริ่มต้น = high-Z = เงียบ */
}

static void Buzzer_On(void)
{
    PORTA->PCR[BUZZ_PIN] = PORT_PCR_MUX(1);
    PTA->PCOR  = (1u << BUZZ_PIN);
    PTA->PDDR |= (1u << BUZZ_PIN);          /* output LOW */
}

static void Buzzer_Off(void)
{
    PTA->PDDR &= ~(1u << BUZZ_PIN);         /* input = high-Z */
}

/*===========================================================================
 *  Buzzer - โหมด FTM0_CH6 (สร้างความถี่โน้ตด้วยฮาร์ดแวร์)
 *  PTA1 ALT3 = FTM0_CH6
 *  f_out = BUS_CLK / (prescaler * (MOD+1)) , เลือก prescaler = 64
 *===========================================================================*/
#define FTM_PRESCALE_DIV   64u
#define TONE_MOD(hz)       ((uint16_t)((BUS_CLK_HZ / FTM_PRESCALE_DIV / (hz)) - 1u))

static void Buzzer_NoTone(void);        /* forward declaration */

static void Buzzer_InitFTM(void)
{
    SIM->SCGC5 |= SIM_SCGC5_PORTA_MASK;
    SIM->SCGC6 |= SIM_SCGC6_FTM0_MASK;

    FTM0->MODE |= FTM_MODE_WPDIS_MASK;      /* ปลดล็อค write protection */
    FTM0->SC    = 0u;                       /* หยุดนาฬิกาไว้ก่อน */
    FTM0->CNT   = 0u;
    FTM0->CNTIN = 0u;

    /* Edge-aligned PWM, high-true pulses */
    FTM0->CONTROLS[6].CnSC = FTM_CnSC_MSB_MASK | FTM_CnSC_ELSB_MASK;
}

static void Buzzer_Tone(uint32_t hz)
{
    uint16_t mod;

    if (hz == 0u) { Buzzer_NoTone(); return; }

    mod = TONE_MOD(hz);
    FTM0->SC  = 0u;                         /* หยุดก่อนแก้ MOD */
    FTM0->CNT = 0u;
    FTM0->MOD = mod;
    FTM0->CONTROLS[6].CnV = (uint16_t)(mod / 2u);    /* duty 50% */

    PORTA->PCR[BUZZ_PIN] = PORT_PCR_MUX(3); /* ต่อขาเข้า FTM0_CH6 */
    FTM0->SC = FTM_SC_CLKS(1) | FTM_SC_PS(6);        /* system clk, /64 */
}

static void Buzzer_NoTone(void)
{
    FTM0->SC = 0u;
    PORTA->PCR[BUZZ_PIN] = PORT_PCR_MUX(1);
    PTA->PDDR &= ~(1u << BUZZ_PIN);         /* high-Z = เงียบสนิท */
}

/*===========================================================================
 *  7-Segment  (74HC595 x 2, bit-bang)
 *===========================================================================*/
static void SevenSeg_Init(void)
{
    SIM->SCGC5 |= SIM_SCGC5_PORTB_MASK | SIM_SCGC5_PORTC_MASK;

    PORTB->PCR[LATCH_PIN] = PORT_PCR_MUX(1);
    PORTC->PCR[CLK_PIN]   = PORT_PCR_MUX(1);
    PORTC->PCR[DATA_PIN]  = PORT_PCR_MUX(1);

    PTB->PDDR |= (1u << LATCH_PIN);
    PTC->PDDR |= (1u << CLK_PIN) | (1u << DATA_PIN);

    PTB->PCOR = (1u << LATCH_PIN);
    PTC->PCOR = (1u << CLK_PIN);
}

/* ส่ง 1 ไบต์เข้า shift register แบบ MSB first */
static void SevenSeg_ShiftByte(uint8_t b)
{
    int8_t i;
    for (i = 7; i >= 0; i--) {
        if (b & (1u << i)) PTC->PSOR = (1u << DATA_PIN);
        else               PTC->PCOR = (1u << DATA_PIN);

        PTC->PSOR = (1u << CLK_PIN);        /* ขอบขาขึ้น = เลื่อนบิต */
        PTC->PCOR = (1u << CLK_PIN);
    }
}

/* แสดงตัวเลข 1 หลัก (pos = 0..3 จากซ้ายไปขวา) */
static void SevenSeg_ShowDigit(uint8_t pos, uint8_t numeral)
{
    PTB->PCOR = (1u << LATCH_PIN);          /* ล็อคเอาต์พุตไว้ระหว่างเลื่อนบิต */

    /* ไบต์แรกไปจบที่ตัวปลายทาง (segments) ไบต์ที่สองอยู่ตัวแรก (digit select) */
    SevenSeg_ShiftByte(SEGMENTS[numeral]);
    SevenSeg_ShiftByte(DIGIT_SELECT[pos]);

    PTB->PSOR = (1u << LATCH_PIN);          /* อัพเดตเอาต์พุตพร้อมกันทั้งสองตัว */
}

static void SevenSeg_Blank(void)
{
    PTB->PCOR = (1u << LATCH_PIN);
    SevenSeg_ShiftByte(0xFF);               /* ดับทุกเซกเมนต์ */
    SevenSeg_ShiftByte(0x00);
    PTB->PSOR = (1u << LATCH_PIN);
}

/* แตกเลข 0..9999 ลงบัฟเฟอร์ 4 หลัก */
static void SevenSeg_SetNumber(uint16_t value)
{
    if (value > 9999u) value = 9999u;
    g_disp[0] = (uint8_t)((value / 1000u) % 10u);
    g_disp[1] = (uint8_t)((value / 100u)  % 10u);
    g_disp[2] = (uint8_t)((value / 10u)   % 10u);
    g_disp[3] = (uint8_t)( value          % 10u);
}

/*===========================================================================
 *  PIT Timer
 *  LDVAL = (คาบที่ต้องการ x bus clock) - 1
 *  ch0 = 2 ms -> multiplex 7-segment  (119999 @ 60 MHz)
 *  ch1 = 1 ms -> system tick          ( 59999 @ 60 MHz)
 *===========================================================================*/
#define PIT_LDVAL_MS(ms)   (((BUS_CLK_HZ / 1000u) * (ms)) - 1u)

static void PIT_Init(void)
{
    SIM->SCGC6 |= SIM_SCGC6_PIT_MASK;
    PIT->MCR = 0u;                          /* เปิดโมดูล, เดินต่อขณะ debug */

    PIT->CHANNEL[0].LDVAL = PIT_LDVAL_MS(2);
    PIT->CHANNEL[0].TCTRL = PIT_TCTRL_TIE_MASK | PIT_TCTRL_TEN_MASK;

    PIT->CHANNEL[1].LDVAL = PIT_LDVAL_MS(1);
    PIT->CHANNEL[1].TCTRL = PIT_TCTRL_TIE_MASK | PIT_TCTRL_TEN_MASK;

    NVIC_SetPriority(PIT0_IRQn, 1);         /* จอภาพสำคัญ -> priority สูงกว่า */
    NVIC_SetPriority(PIT1_IRQn, 2);
    NVIC_EnableIRQ(PIT0_IRQn);
    NVIC_EnableIRQ(PIT1_IRQn);
}

/* ISR ของ PIT0 : หมุนแสดงผลทีละหลัก */
void PIT0_IRQHandler(void)
{
    static uint8_t pos = 0u;

    PIT->CHANNEL[0].TFLG = PIT_TFLG_TIF_MASK;   /* เคลียร์ flag (w1c) */

    SevenSeg_ShowDigit(pos, g_disp[pos]);
    pos = (uint8_t)((pos + 1u) & 0x03u);
}

/* ISR ของ PIT1 : ฐานเวลา 1 ms */
void PIT1_IRQHandler(void)
{
    PIT->CHANNEL[1].TFLG = PIT_TFLG_TIF_MASK;
    g_tick_ms++;
}

/*===========================================================================
 *  PORTB External Interrupt  (S1, S2, S3 และต่อไปคือ PIR ที่ PTB9)
 *===========================================================================*/
static volatile uint8_t  g_btn_event = 0u;      /* 1=S1, 2=S2, 3=S3 */
static volatile uint32_t g_btn_count = 0u;

static void Button_IRQ_Init(void)
{
    SIM->SCGC5 |= SIM_SCGC5_PORTB_MASK;

    /* MUX=GPIO, pull-up เปิด, IRQC=1010b = ขอบขาลง (falling edge) */
    PORTB->PCR[S1_PIN] = PORT_PCR_MUX(1) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK
                       | PORT_PCR_IRQC(0x0A);
    PORTB->PCR[S2_PIN] = PORT_PCR_MUX(1) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK
                       | PORT_PCR_IRQC(0x0A);
    PORTB->PCR[S3_PIN] = PORT_PCR_MUX(1) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK
                       | PORT_PCR_IRQC(0x0A);

    PTB->PDDR &= ~((1u<<S1_PIN) | (1u<<S2_PIN) | (1u<<S3_PIN));

    PORTB->ISFR = 0xFFFFFFFFu;              /* เคลียร์ flag ค้าง */
    NVIC_SetPriority(PORTB_IRQn, 3);
    NVIC_EnableIRQ(PORTB_IRQn);
}

/* ISR ตัวเดียว รับทั้ง S1/S2/S3 (และ PIR ที่ PTB9 ในอนาคต)
 * ข้อสำคัญ : ISR ต้องสั้น ทำแค่ตั้ง flag แล้วให้ main loop ไปจัดการต่อ */
void PORTB_IRQHandler(void)
{
    uint32_t flags = PORTB->ISFR;
    static uint32_t last_ms = 0u;

    /* software debounce : ทิ้ง event ที่ห่างจากครั้งก่อนไม่ถึง 30 ms */
    if ((g_tick_ms - last_ms) >= 30u) {
        if      (flags & (1u << S1_PIN)) { g_btn_event = 1u; g_btn_count++; }
        else if (flags & (1u << S2_PIN)) { g_btn_event = 2u; g_btn_count++; }
        else if (flags & (1u << S3_PIN)) { g_btn_event = 3u; g_btn_count++; }
        last_ms = g_tick_ms;
    }

    PORTB->ISFR = flags;                    /* เคลียร์ flag ทุกครั้ง (w1c) */
}

/*===========================================================================
 *  MAIN
 *===========================================================================*/
int main(void)
{
    /* ถ้าโปรเจคสร้างจาก SDK board files ให้เปิดบรรทัดนี้เพื่อตั้ง core 120MHz / bus 60MHz
     * ไม่เรียก = วิ่งที่ clock เริ่มต้นหลังรีเซ็ต (~20.97 MHz) ซึ่งต้องแก้ BUS_CLK_HZ ตาม
     *
     *   BOARD_InitBootClocks();
     */

/*---------------------------------------------------------------------------
 *  STAGE 1 : ทดสอบ LED - ไล่ไฟ LED1 -> LED4 แล้วกระพริบพร้อมกัน
 *  คาดหวัง : LED ติดทีละดวงจากซ้ายไปขวา แล้วกระพริบพร้อมกัน 3 ครั้ง วนซ้ำ
 *-------------------------------------------------------------------------*/
#if (TEST_STAGE == 1)
    uint8_t i, k;
    LED_Init();

    while (1) {
        for (i = 1; i <= 4; i++) {          /* ไล่ทีละดวง */
            LED_AllOff();
            LED_On(i);
            delay_ms(250);
        }
        LED_AllOff();
        delay_ms(300);

        for (k = 0; k < 3; k++) {           /* กระพริบพร้อมกัน */
            PTD->PCOR = LED_ALL;  delay_ms(150);
            PTD->PSOR = LED_ALL;  delay_ms(150);
        }
        delay_ms(500);
    }

/*---------------------------------------------------------------------------
 *  STAGE 2 : ทดสอบปุ่ม (polling)
 *  คาดหวัง : กด S1 -> LED1 ติด, S2 -> LED2 ติด, S3 -> LED3 ติด, ปล่อย = ดับ
 *-------------------------------------------------------------------------*/
#elif (TEST_STAGE == 2)
    LED_Init();
    Button_Init();

    while (1) {
        if (Button_IsPressed(S1_PIN)) LED_On(1); else LED_Off(1);
        if (Button_IsPressed(S2_PIN)) LED_On(2); else LED_Off(2);
        if (Button_IsPressed(S3_PIN)) LED_On(3); else LED_Off(3);
    }

/*---------------------------------------------------------------------------
 *  STAGE 3 : ทดสอบ Buzzer แบบ GPIO + พิสูจน์ปัญหา PNP
 *  คาดหวัง : กด S1 = buzzer ดัง + LED4 ติด / ปล่อย = เงียบสนิท
 *            กด S2 = ทดลองปิดแบบผิด (ขับ HIGH) -> ถ้ายังได้ยินเสียงหวี่ๆ
 *                    แปลว่าเจอปัญหาตาม WARNING #2 จริง
 *-------------------------------------------------------------------------*/
#elif (TEST_STAGE == 3)
    LED_Init();
    Button_Init();
    Buzzer_InitGPIO();

    while (1) {
        if (Button_IsPressed(S1_PIN)) {
            Buzzer_On();  LED_On(4);
        } else if (Button_IsPressed(S2_PIN)) {
            /* วิธีปิดแบบผิด : ขับ HIGH ที่ 3.3V */
            PORTA->PCR[BUZZ_PIN] = PORT_PCR_MUX(1);
            PTA->PDDR |= (1u << BUZZ_PIN);
            PTA->PSOR  = (1u << BUZZ_PIN);
            LED_Off(4);
        } else {
            Buzzer_Off();  LED_Off(4);      /* วิธีปิดที่ถูก : high-Z */
        }
    }

/*---------------------------------------------------------------------------
 *  STAGE 4 : ทดสอบ 7-Segment แบบ bit-bang (ยังไม่ใช้ timer)
 *  คาดหวัง : แสดงเลข 1234 นิ่งๆ ไม่กระพริบ
 *  ถ้าเลขกลับด้าน -> สลับลำดับใน DIGIT_SELECT[]
 *  ถ้ารูปตัวเลขเพี้ยน -> ตรวจ SEGMENTS[] และขั้ว active-low
 *-------------------------------------------------------------------------*/
#elif (TEST_STAGE == 4)
    uint8_t pos;
    const uint8_t value[4] = {1, 2, 3, 4};

    SevenSeg_Init();

    while (1) {
        for (pos = 0; pos < 4; pos++) {
            SevenSeg_ShowDigit(pos, value[pos]);
            delay_ms(2);                    /* รีเฟรชหลักละ 2 ms */
        }
    }

/*---------------------------------------------------------------------------
 *  STAGE 5 : ทดสอบ PIT Timer - multiplex 7-seg ใน ISR + นาฬิกาจับเวลา
 *  คาดหวัง : 7-Segment นับวินาที 0000 ขึ้นไปเรื่อยๆ นิ่งไม่กระพริบ
 *            LED1 กระพริบ 1 Hz (ติด 0.5 วิ ดับ 0.5 วิ)
 *            main loop ว่าง ไม่มี delay เลย <- นี่คือข้อดีของการใช้ timer
 *-------------------------------------------------------------------------*/
#elif (TEST_STAGE == 5)
    uint32_t next_sec = 1000u;
    uint16_t seconds  = 0u;

    LED_Init();
    SevenSeg_Init();
    PIT_Init();

    while (1) {
        if (g_tick_ms >= next_sec) {
            next_sec += 1000u;
            seconds++;
            SevenSeg_SetNumber(seconds);
            LED_Toggle(1);
        }
    }

/*---------------------------------------------------------------------------
 *  STAGE 6 : ทดสอบ Interrupt + debounce (จำลอง PIR ด้วย S3)
 *  คาดหวัง : กดปุ่มแต่ละครั้ง เลขบน 7-Segment เพิ่มทีละ 1 เป๊ะๆ
 *            (ถ้าเพิ่มทีละ 2-3 แปลว่า debounce ไม่ทำงาน)
 *            S1 -> LED1 ติด, S2 -> LED2 ติด, S3 -> LED3+LED4 ติด (จำลอง PIR)
 *-------------------------------------------------------------------------*/
#elif (TEST_STAGE == 6)
    LED_Init();
    SevenSeg_Init();
    PIT_Init();
    Button_IRQ_Init();

    while (1) {
        uint8_t ev = g_btn_event;

        if (ev != 0u) {
            g_btn_event = 0u;               /* เคลียร์ flag */
            LED_AllOff();

            switch (ev) {
                case 1: LED_On(1); break;
                case 2: LED_On(2); break;
                case 3: LED_On(3); LED_On(4); break;   /* จำลอง PIR trigger */
                default: break;
            }
            SevenSeg_SetNumber((uint16_t)g_btn_count);
        }
    }

/*---------------------------------------------------------------------------
 *  STAGE 7 : ทดสอบ Buzzer melody ด้วย FTM0_CH6
 *  คาดหวัง : เล่นสเกล C major ขึ้น-ลง 1 รอบ แล้วเล่นไซเรน 3 รอบ วนซ้ำ
 *            7-Segment แสดงหมายเลขโน้ตที่กำลังเล่น
 *  ถ้าเสียงเพี้ยนความถี่ -> ค่า BUS_CLK_HZ ไม่ตรงกับของจริง
 *-------------------------------------------------------------------------*/
#elif (TEST_STAGE == 7)
    /* C4 D4 E4 F4 G4 A4 B4 C5 */
    static const uint16_t scale[8] = {262, 294, 330, 349, 392, 440, 494, 523};
    uint8_t  i;
    uint16_t f;

    LED_Init();
    SevenSeg_Init();
    PIT_Init();
    Buzzer_InitFTM();

    while (1) {
        for (i = 0; i < 8; i++) {           /* ไล่สเกลขึ้น */
            SevenSeg_SetNumber((uint16_t)(i + 1u));
            LED_Toggle(1);
            Buzzer_Tone(scale[i]);
            delay_ms(300);
            Buzzer_NoTone();
            delay_ms(60);
        }
        for (i = 8; i > 0; i--) {           /* ไล่สเกลลง */
            SevenSeg_SetNumber(i);
            LED_Toggle(2);
            Buzzer_Tone(scale[i - 1u]);
            delay_ms(300);
            Buzzer_NoTone();
            delay_ms(60);
        }

        delay_ms(500);

        for (i = 0; i < 3; i++) {           /* ไซเรนกวาดความถี่ */
            for (f = 400; f < 1200; f += 20) { Buzzer_Tone(f); delay_ms(5); }
            for (f = 1200; f > 400; f -= 20) { Buzzer_Tone(f); delay_ms(5); }
        }
        Buzzer_NoTone();
        delay_ms(1000);
    }

#else
    #error "TEST_STAGE ต้องเป็น 1 ถึง 7"
#endif
}
