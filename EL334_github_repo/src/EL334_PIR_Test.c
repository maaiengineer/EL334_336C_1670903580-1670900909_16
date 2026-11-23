/*=============================================================================
 *  EL334 - PIR wiring test
 *  FRDM-K64F + Multi-Function Shield
 *
 *  This does one job only: it shows the state of the PIR output pin on the
 *  LEDs, with no state machine, no timers and no interrupts in the way.
 *
 *      LED1  follows the pin live   : ON while the PIR output is HIGH
 *      LED4  latches                : turns ON the first time the pin ever
 *                                     goes HIGH and stays on, so a pulse too
 *                                     short to see is still caught
 *
 *  PIR OUT goes to the signal pin (right column) of the shield's D6 row,
 *  which reaches the MCU at PTC2.
 *
 *  How to read the result
 *  ----------------------
 *   LED1 lights when you wave      -> wiring and module are fine, the problem
 *                                     is in the main program
 *   LED1 never lights, LED4 does   -> the module pulses very briefly: turn the
 *                                     Tx (time) pot up a little
 *   both stay off                  -> OUT is on the wrong pin, the module has
 *                                     no power, or the sensitivity pot is at
 *                                     minimum
 *   both on straight away and stay -> still warming up, or Tx is set very long
 *
 *  Give the module a full minute of power before judging anything: an
 *  HC-SR501 outputs nonsense until it has settled to the room.
 *===========================================================================*/

#include "MK64F12.h"
#include <stdint.h>

#define PIR_PIN    2u      /* PTC2  = Arduino D6 */
#define LED1_PIN   1u      /* PTD1  = silk D1    */
#define LED4_PIN   0u      /* PTD0  = silk D4    */

int main(void)
{
    uint32_t led_mask = (1u << LED1_PIN) | (1u << LED4_PIN);

    SIM->SCGC5 |= SIM_SCGC5_PORTC_MASK | SIM_SCGC5_PORTD_MASK;

    /* PIR input. The module drives the line, so no pull resistor. */
    PORTC->PCR[PIR_PIN] = PORT_PCR_MUX(1);
    PTC->PDDR &= ~(1u << PIR_PIN);

    /* LEDs are active low */
    PORTD->PCR[LED1_PIN] = PORT_PCR_MUX(1);
    PORTD->PCR[LED4_PIN] = PORT_PCR_MUX(1);
    PTD->PDDR |= led_mask;
    PTD->PSOR  = led_mask;                      /* both off */

    for (;;) {
        if (PTC->PDIR & (1u << PIR_PIN)) {
            PTD->PCOR = (1u << LED1_PIN);       /* live state  */
            PTD->PCOR = (1u << LED4_PIN);       /* latch, never cleared */
        } else {
            PTD->PSOR = (1u << LED1_PIN);
        }
    }
}
