/*
 *  pwm.c
 *
 *  To run this project, set low FUSE bit 0b11100001,
 *  which means no CKDIV8, and use PLL for the system clock.
 *  The system clock will be 16MHz. 
 */
#define F_CPU (16000000UL)

#include <avr/io.h>
#include <avr/iotn85.h>
#include <avr/interrupt.h>
#include <util/delay.h>

const uint8_t sin_table[] = {
  140,153,165,177,189,199,210,219,227,235,241,246,250,253,255,255,
  254,252,248,243,238,231,223,214,204,193,182,170,158,145,133,120,
  107,95, 83, 71, 60, 49, 40, 31, 23, 16, 11, 6,  3,  1,  0,  0,
  2,  5,  9,  15, 21, 29, 38, 47, 57, 68, 80, 92, 104,117
};

uint16_t freq = 420;
uint16_t period;
uint16_t i = 0;

/*
 * called every sampling cycle
 */
ISR(TIM0_COMPA_vect)
{
  OCR1B = sin_table[((uint16_t)62 * i) / period];
  if (i < period - 1)
    i++;
  else
    i = 0;
}

int main(void)
{
  period = 20000 / freq;
  /* set DDRB -- Data Direction Registor for port B
   * Set PB4, PB3 as OUTPUT
   */
  DDRB = (1<<DDB4)|(1<<DDB3);

  PORTB = 0x00;

  PLLCSR = (1<<PCKE)|(1<<PLLE); /* Select PLL clock for TC1.ck */
  OCR1C = 0xFF; /* TOP value */
  OCR1B = 0x00;
  GTCCR = (1<<PWM1B)|(1<<COM1B1)|(0<<COM1B0);
  TCCR1 = 0x01; /* Start TC1. CS13-10: 0001(PCK(64MHz) in asynchronous mode) */
  TCNT1 = 0;

  OCR0A  = (uint8_t)(F_CPU / 8 / 20000) - 1;
  TCCR0A = 0b00000010; /* Start TC0 as interval timer at 2MHz */
  TCCR0B = 0b00000010;

  TIMSK = (1<<OCIE0A);

  sei();

  while (1) {
  }

  return 0;
}
