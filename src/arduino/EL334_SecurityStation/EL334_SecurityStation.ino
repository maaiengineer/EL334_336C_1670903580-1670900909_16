/*=============================================================================
 *  EL334 Mini-Project  -  Smart Security / Access Station
 *  Arduino version (Arduino UNO / Nano + Multi-Function Shield)
 *
 *  This sketch mirrors the FRDM-K64F version feature for feature, so the two
 *  can be compared side by side. The structure is identical: one state
 *  machine, one timer-driven display, interrupt-driven inputs, and a
 *  non-blocking melody player. There is no delay() anywhere.
 *
 *---------------------------------------------------------------------------
 *  TIMERS AND INTERRUPTS
 *---------------------------------------------------------------------------
 *  Timer1 (CTC, 2 ms)  -> multiplexes the four-digit display
 *                         (the K64F version uses PIT channel 0)
 *  Timer0 via millis() -> system tick and switch debounce
 *                         (the K64F version uses PIT channel 1)
 *  Timer2 via tone()   -> buzzer frequency generation
 *                         (the K64F version uses FTM0 channel 6)
 *  PCINT1 vector       -> switches S1, S2, S3 on A1, A2, A3
 *  PCINT2 vector       -> PIR sensor on D6
 *                         (the K64F version uses the PORTB and PORTC vectors)
 *
 *---------------------------------------------------------------------------
 *  WIRING
 *---------------------------------------------------------------------------
 *  Everything except the PIR is already on the shield.
 *
 *  The PIR plugs into the shield's 3-pin external-device block at the bottom
 *  right (12 pins, 3 columns x 4 rows):
 *      columns, left to right : GND, +5 V, signal
 *      rows, top to bottom    : D5, D6, D9, A5
 *  Put the PIR's OUT on the signal pin of the D6 row, VCC on the middle
 *  column and GND on the left column.
 *
 *  On a 5 V Arduino the shield needs no modification: jumper J2 may stay on,
 *  and the buzzer's PNP driver switches off cleanly because the pin reaches a
 *  full 5 V. Both of those are problems only on the 3.3 V FRDM-K64F.
 *===========================================================================*/

/*===========================================================================
 *  1. BUILD CONFIGURATION
 *===========================================================================*/

/* 0 = no PIR fitted yet, switch S3 stands in for a motion event while ARMED
 * 1 = real HC-SR501 on the shield's D6 header                              */
#define PIR_USE_HARDWARE   1

/*===========================================================================
 *  2. PIN DEFINITIONS
 *===========================================================================*/
const uint8_t LED_PIN[4] = { 13, 12, 11, 10 };   /* LED1..LED4, active LOW */

const uint8_t S1_PIN   = A1;
const uint8_t S2_PIN   = A2;
const uint8_t S3_PIN   = A3;

const uint8_t BUZZ_PIN = 3;

const uint8_t LATCH_PIN = 4;    /* 74HC595 latch clock */
const uint8_t CLK_PIN   = 7;    /* 74HC595 shift clock */
const uint8_t DATA_PIN  = 8;    /* 74HC595 serial data */

const uint8_t PIR_PIN   = 6;    /* shield's D6 header */

/*===========================================================================
 *  3. APPLICATION PARAMETERS
 *===========================================================================*/
const uint8_t  EXIT_DELAY_SEC      = 10;   /* time to leave after arming     */
const uint8_t  ENTRY_DELAY_SEC     = 15;   /* time to disarm after detection */
const uint8_t  CODE_LEN            = 4;
const uint8_t  DEBOUNCE_RELEASE_MS = 30;   /* quiet time before a new press  */
const uint16_t UNLOCK_SHOW_MS      = 1500; /* how long "OPEn" stays up       */

/* Secret code as switch numbers: S1, S3, S2, S1 */
const uint8_t SECRET_CODE[CODE_LEN] = { 1, 3, 2, 1 };

/*===========================================================================
 *  4. SHARED STATE
 *===========================================================================*/
enum AppState {
  ST_DISARMED = 0,
  ST_ARMING,
  ST_ARMED,
  ST_TRIGGERED,
  ST_ALARM,
  ST_UNLOCK
};

volatile uint8_t g_segbuf[4]  = { 0xFF, 0xFF, 0xFF, 0xFF };
volatile uint8_t g_btn_event  = 0;   /* 1 = S1, 2 = S2, 3 = S3, 0 = none */
volatile uint8_t g_pir_event  = 0;
volatile uint8_t g_btn_armed  = 1;   /* ready to accept a new press      */
volatile uint8_t g_release_t  = 0;   /* counts 2 ms ticks of quiet       */

AppState g_state      = ST_DISARMED;
uint32_t g_state_ms   = 0;           /* millis() when the state began    */
uint8_t  g_remain_sec = 0;
uint8_t  g_code_idx   = 0;

/*===========================================================================
 *  5. SEVEN-SEGMENT CHARACTER TABLE
 *  Active-low patterns, bit order DP G F E D C B A. A 0 bit lights a segment.
 *===========================================================================*/
const uint8_t SEG_DIGIT[10] = {
  0xC0, 0xF9, 0xA4, 0xB0, 0x99, 0x92, 0x82, 0xF8, 0x80, 0x90
};

const uint8_t SEG_BLANK = 0xFF;
const uint8_t SEG_DASH  = 0xBF;
const uint8_t SEG_A     = 0x88;
const uint8_t SEG_L     = 0xC7;
const uint8_t SEG_r     = 0xAF;
const uint8_t SEG_n     = 0xAB;
const uint8_t SEG_O     = 0xC0;
const uint8_t SEG_P     = 0x8C;
const uint8_t SEG_E     = 0x86;

/* Digit-enable masks, left to right. Reorder if the display runs backwards. */
const uint8_t DIGIT_SELECT[4] = { 0xF1, 0xF2, 0xF4, 0xF8 };

/*===========================================================================
 *  6. LED MODULE                                             (active LOW)
 *===========================================================================*/
void LED_Init() {
  for (uint8_t i = 0; i < 4; i++) {
    pinMode(LED_PIN[i], OUTPUT);
    digitalWrite(LED_PIN[i], HIGH);          /* HIGH = off */
  }
}

void LED_On(uint8_t n)  { if (n >= 1 && n <= 4) digitalWrite(LED_PIN[n - 1], LOW);  }
void LED_Off(uint8_t n) { if (n >= 1 && n <= 4) digitalWrite(LED_PIN[n - 1], HIGH); }

void LED_AllOff() {
  for (uint8_t i = 1; i <= 4; i++) LED_Off(i);
}

/* Light LED1..LEDn to show how many code keys have been entered. */
void LED_ShowProgress(uint8_t n) {
  for (uint8_t i = 1; i <= 4; i++) {
    if (i <= n) LED_On(i); else LED_Off(i);
  }
}

/* Returns true for the first half of every period_ms window. */
bool LED_BlinkPhase(uint16_t period_ms) {
  return (millis() % period_ms) < (uint32_t)(period_ms / 2);
}

/* Running light used by the alarm state. */
void LED_Chase(uint16_t step_ms) {
  uint8_t pos = (millis() / step_ms) & 0x03;
  LED_AllOff();
  LED_On(pos + 1);
}

/*===========================================================================
 *  7. SEVEN-SEGMENT MODULE                      (two cascaded 74HC595 chips)
 *===========================================================================*/
void SevenSeg_Init() {
  pinMode(LATCH_PIN, OUTPUT);
  pinMode(CLK_PIN,   OUTPUT);
  pinMode(DATA_PIN,  OUTPUT);
  digitalWrite(LATCH_PIN, LOW);
  digitalWrite(CLK_PIN,   LOW);
}

/* Drive one digit position with one raw segment pattern.
 * The first byte sent travels furthest and lands in the segment register;
 * the second stays in the nearer register and selects the digit. */
void SevenSeg_ShowRaw(uint8_t pos, uint8_t pattern) {
  digitalWrite(LATCH_PIN, LOW);                /* hold outputs while shifting */
  shiftOut(DATA_PIN, CLK_PIN, MSBFIRST, pattern);
  shiftOut(DATA_PIN, CLK_PIN, MSBFIRST, DIGIT_SELECT[pos & 0x03]);
  digitalWrite(LATCH_PIN, HIGH);               /* latch both chips at once */
}

void SevenSeg_SetRaw(uint8_t d0, uint8_t d1, uint8_t d2, uint8_t d3) {
  g_segbuf[0] = d0;  g_segbuf[1] = d1;
  g_segbuf[2] = d2;  g_segbuf[3] = d3;
}

void SevenSeg_Blank()  { SevenSeg_SetRaw(SEG_BLANK, SEG_BLANK, SEG_BLANK, SEG_BLANK); }
void SevenSeg_Dashes() { SevenSeg_SetRaw(SEG_DASH,  SEG_DASH,  SEG_DASH,  SEG_DASH);  }

/* Right-aligned unsigned number with leading blanks. */
void SevenSeg_SetNumber(uint16_t v) {
  uint8_t buf[4];

  if (v > 9999) v = 9999;

  for (int8_t i = 3; i >= 0; i--) {
    buf[i] = SEG_DIGIT[v % 10];
    v /= 10;
  }
  for (uint8_t i = 0; i < 3; i++) {            /* blank the leading zeros */
    if (buf[i] == SEG_DIGIT[0]) buf[i] = SEG_BLANK;
    else break;
  }
  SevenSeg_SetRaw(buf[0], buf[1], buf[2], buf[3]);
}

/*===========================================================================
 *  8. BUZZER MODULE                                   (tone() uses Timer2)
 *===========================================================================*/
struct Note {
  uint16_t freq;     /* Hz, 0 = rest      */
  uint16_t ms;       /* duration, 0 = end */
};

const Note* g_song     = NULL;
uint8_t     g_song_idx = 0;
bool        g_song_loop = false;
uint32_t    g_note_end = 0;

void Buzzer_Init() {
  pinMode(BUZZ_PIN, OUTPUT);
  digitalWrite(BUZZ_PIN, HIGH);                /* HIGH = silent (PNP driver) */
}

void Buzzer_NoTone() {
  noTone(BUZZ_PIN);
  digitalWrite(BUZZ_PIN, HIGH);
}

void Buzzer_Stop() {
  g_song = NULL;
  Buzzer_NoTone();
}

void Buzzer_StartNote() {
  if (g_song == NULL) return;

  if (g_song[g_song_idx].ms == 0) {            /* end-of-song marker */
    if (g_song_loop) {
      g_song_idx = 0;
    } else {
      Buzzer_Stop();
      return;
    }
  }
  if (g_song[g_song_idx].freq == 0) Buzzer_NoTone();
  else                              tone(BUZZ_PIN, g_song[g_song_idx].freq);

  g_note_end = millis() + g_song[g_song_idx].ms;
}

void Buzzer_Play(const Note* song, bool loop) {
  g_song      = song;
  g_song_idx  = 0;
  g_song_loop = loop;
  Buzzer_StartNote();
}

/* Called from loop() as often as possible. Never blocks. */
void Buzzer_Task() {
  if (g_song == NULL) return;
  if ((int32_t)(millis() - g_note_end) < 0) return;

  g_song_idx++;
  Buzzer_StartNote();
}

/*--- Melody definitions ---------------------------------------------------*/
const Note SONG_BEEP[]    = { {1200,  60}, {0, 0} };
const Note SONG_BEEP_HI[] = { {2000,  70}, {0, 0} };
const Note SONG_ARMED[]   = { { 784, 100}, {1047, 160}, {0, 0} };
const Note SONG_OK[]      = { { 523, 110}, { 659, 110}, {784, 220}, {0, 0} };
const Note SONG_ERROR[]   = { { 220, 180}, {   0,  60}, {175, 260}, {0, 0} };
const Note SONG_SIREN[]   = { { 880, 180}, { 660, 180}, {0, 0} };   /* looped */

/*===========================================================================
 *  9. SWITCH MODULE                        (active LOW, pin-change interrupt)
 *  A1, A2 and A3 are PCINT9, PCINT10 and PCINT11, all on the PCINT1 vector.
 *===========================================================================*/
void Button_Init() {
  pinMode(S1_PIN, INPUT_PULLUP);
  pinMode(S2_PIN, INPUT_PULLUP);
  pinMode(S3_PIN, INPUT_PULLUP);

  PCICR  |= (1 << PCIE1);
  PCMSK1 |= (1 << PCINT9) | (1 << PCINT10) | (1 << PCINT11);
}

/* Read and clear the pending switch event in one step. */
uint8_t Button_GetEvent() {
  uint8_t ev = g_btn_event;
  if (ev != 0) g_btn_event = 0;
  return ev;
}

bool Button_AllReleased() {
  return digitalRead(S1_PIN) && digitalRead(S2_PIN) && digitalRead(S3_PIN);
}

/*===========================================================================
 * 10. PIR MODULE                         (HC-SR501 on D6 = PCINT22, PCINT2)
 *===========================================================================*/
void PIR_Init() {
#if (PIR_USE_HARDWARE == 1)
  pinMode(PIR_PIN, INPUT);                     /* module drives the line */
  PCICR  |= (1 << PCIE2);
  PCMSK2 |= (1 << PCINT22);
#endif
}

uint8_t PIR_GetEvent() {
  uint8_t ev = g_pir_event;
  if (ev != 0) g_pir_event = 0;
  return ev;
}

/*===========================================================================
 * 11. TIMER1  -  display refresh every 2 ms, plus the release half of the
 *                switch debounce. CTC mode, prescaler 8.
 *                OCR1A = 16 MHz / 8 / 500 Hz - 1 = 3999
 *===========================================================================*/
void Timer1_Init() {
  noInterrupts();
  TCCR1A = 0;
  TCCR1B = (1 << WGM12) | (1 << CS11);         /* CTC, clk/8 */
  TCNT1  = 0;
  OCR1A  = 3999;                               /* 2 ms */
  TIMSK1 = (1 << OCIE1A);
  interrupts();
}

ISR(TIMER1_COMPA_vect) {
  static uint8_t pos = 0;

  SevenSeg_ShowRaw(pos, g_segbuf[pos]);
  pos = (pos + 1) & 0x03;

  /* A new press is only accepted after every switch has been released
   * continuously for DEBOUNCE_RELEASE_MS. This rejects both the press
   * bounce and the release bounce of the contacts. One tick is 2 ms. */
  if (Button_AllReleased()) {
    if (g_release_t < (DEBOUNCE_RELEASE_MS / 2)) {
      g_release_t++;
      if (g_release_t >= (DEBOUNCE_RELEASE_MS / 2)) g_btn_armed = 1;
    }
  } else {
    g_release_t = 0;
  }
}

/*===========================================================================
 * 12. PIN-CHANGE INTERRUPT HANDLERS
 *  Both stay short: they only set a flag. Every decision is made in loop().
 *  A pin-change interrupt fires on BOTH edges, so each handler checks the
 *  level to keep only the edge it wants.
 *===========================================================================*/

/* Switches S1, S2, S3 - keep the falling edge (the line is now LOW). */
ISR(PCINT1_vect) {
  if (!g_btn_armed) return;                    /* previous press not released */

  if      (digitalRead(S1_PIN) == LOW) g_btn_event = 1;
  else if (digitalRead(S2_PIN) == LOW) g_btn_event = 2;
  else if (digitalRead(S3_PIN) == LOW) g_btn_event = 3;
  else return;                                 /* a release, ignore */

  g_btn_armed = 0;                             /* lock until released */
}

#if (PIR_USE_HARDWARE == 1)
/* PIR on D6 - keep the rising edge (motion detected). */
ISR(PCINT2_vect) {
  if (digitalRead(PIR_PIN) == HIGH) g_pir_event = 1;
}
#endif

/*===========================================================================
 * 13. SECRET CODE MODULE
 *===========================================================================*/
void Code_Reset() { g_code_idx = 0; }

/* Returns 0 = still collecting, 1 = complete and correct, 2 = wrong key */
uint8_t Code_AddKey(uint8_t key) {
  if (key != SECRET_CODE[g_code_idx]) {        /* wrong key at this position */
    g_code_idx = 0;
    return 2;
  }
  g_code_idx++;
  if (g_code_idx >= CODE_LEN) {
    g_code_idx = 0;
    return 1;
  }
  return 0;
}

/*===========================================================================
 * 14. STATE MACHINE
 *===========================================================================*/
void App_EnterState(AppState s) {
  g_state    = s;
  g_state_ms = millis();
  Code_Reset();
  LED_AllOff();
  Buzzer_Stop();

  switch (s) {
    case ST_DISARMED:
      g_remain_sec = 0;
      SevenSeg_Dashes();
      break;

    case ST_ARMING:
      g_remain_sec = EXIT_DELAY_SEC;
      SevenSeg_SetNumber(g_remain_sec);
      break;

    case ST_ARMED:
      g_remain_sec = 0;
      SevenSeg_SetRaw(SEG_BLANK, SEG_O, SEG_n, SEG_BLANK);
      Buzzer_Play(SONG_ARMED, false);
      break;

    case ST_TRIGGERED:
      g_remain_sec = ENTRY_DELAY_SEC;
      SevenSeg_SetNumber(g_remain_sec);
      break;

    case ST_ALARM:
      g_remain_sec = 0;
      Buzzer_Play(SONG_SIREN, true);           /* looping siren */
      break;

    case ST_UNLOCK:
      g_remain_sec = 0;
      SevenSeg_SetRaw(SEG_O, SEG_P, SEG_E, SEG_n);
      Buzzer_Play(SONG_OK, false);
      LED_On(1); LED_On(2); LED_On(3); LED_On(4);
      break;
  }
}

/* Shared by TRIGGERED and ALARM: collect code keys and act on the result. */
bool App_HandleCodeEntry(uint8_t key) {
  if (key == 0) return false;

  uint8_t result = Code_AddKey(key);

  if (result == 1) return true;                /* correct, caller transitions */
  if (result == 2) { Buzzer_Play(SONG_ERROR, false); return false; }

  Buzzer_Play(SONG_BEEP, false);               /* accepted one key */
  return false;
}

void State_Disarmed() {
  uint8_t key = Button_GetEvent();

  LED_On(1);                                   /* steady: system is off */

  if (key == 1) App_EnterState(ST_ARMING);
}

void State_Arming() {
  uint8_t  key     = Button_GetEvent();
  uint32_t elapsed = millis() - g_state_ms;

  if (key != 0) {                              /* any key cancels */
    App_EnterState(ST_DISARMED);
    return;
  }

  if (LED_BlinkPhase(500)) LED_On(2); else LED_Off(2);   /* 2 Hz */

  if (elapsed >= (uint32_t)EXIT_DELAY_SEC * 1000UL) {
    App_EnterState(ST_ARMED);
    return;
  }

  uint8_t left = EXIT_DELAY_SEC - (elapsed / 1000UL);
  if (left != g_remain_sec) {                  /* one second has passed */
    g_remain_sec = left;
    SevenSeg_SetNumber(g_remain_sec);
    Buzzer_Play(SONG_BEEP, false);
  }
}

void State_Armed() {
  uint8_t key = Button_GetEvent();

  if (LED_BlinkPhase(2000)) LED_On(2); else LED_Off(2);  /* 0.5 Hz */

#if (PIR_USE_HARDWARE == 0)
  if (key == 3) g_pir_event = 1;               /* S3 stands in for the PIR */
#endif

  if (PIR_GetEvent()) {
    App_EnterState(ST_TRIGGERED);
    return;
  }
  if (key == 1) App_EnterState(ST_TRIGGERED);  /* owner coming home */
}

void State_Triggered() {
  uint8_t  key     = Button_GetEvent();
  uint32_t elapsed = millis() - g_state_ms;

  LED_ShowProgress(g_code_idx);

  if (App_HandleCodeEntry(key)) {
    App_EnterState(ST_UNLOCK);
    return;
  }

  if (elapsed >= (uint32_t)ENTRY_DELAY_SEC * 1000UL) {
    App_EnterState(ST_ALARM);
    return;
  }

  uint8_t left = ENTRY_DELAY_SEC - (elapsed / 1000UL);
  if (left != g_remain_sec) {
    g_remain_sec = left;
    SevenSeg_SetNumber(g_remain_sec);
    /* the warning beep gets higher as the remaining time shrinks */
    if (g_song == NULL) {
      Buzzer_Play((left <= 5) ? SONG_BEEP_HI : SONG_BEEP, false);
    }
  }
}

void State_Alarm() {
  uint8_t key = Button_GetEvent();

  LED_Chase(120);

  if (LED_BlinkPhase(1000)) SevenSeg_SetRaw(SEG_A, SEG_L, SEG_A, SEG_r);
  else                      SevenSeg_Blank();

  if (App_HandleCodeEntry(key)) App_EnterState(ST_UNLOCK);
}

/* Holds the "OPEn" confirmation long enough to be seen and heard. */
void State_Unlock() {
  Button_GetEvent();                           /* swallow stray presses */

  if ((millis() - g_state_ms) >= UNLOCK_SHOW_MS) {
    App_EnterState(ST_DISARMED);
  }
}

void App_Run() {
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
 * 15. SETUP AND LOOP
 *===========================================================================*/
void setup() {
  LED_Init();
  SevenSeg_Init();
  Buzzer_Init();
  Button_Init();
  PIR_Init();
  Timer1_Init();                               /* start the display last */

  App_EnterState(ST_DISARMED);
}

void loop() {
  Buzzer_Task();      /* advance the melody, never blocks */
  App_Run();          /* run the current state handler    */
}
