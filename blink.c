#include <avr/io.h>
#include <util/delay.h>

int main(void)
{
  /* set DDRB -- Data Direction Registor for port B
   * Set PB4 as OUTPUT
   * port  | B7 B6 B5 B4 B3 B2 B1 B0
   * dir   | I  I  I  O  I  I  I  I
   * value | 0  0  0  1  0  0  0  0
   * 0b00010000 = 0x10
   */
  DDRB = 0x10;

  /* set value for port B
   * port  | B7 B6 B5 B4 B3 B2 B1 B0
   * value | 0  0  0  1  0  0  0  0
   * 0b00010000 = 0x10 for ON
   * 0x00000000 = 0x00 for OFF
   */
  PORTB = 0x00;

  /* Toggle ON/OFF every 500ms */
  while (1) {
    PORTB = 0x10;
    _delay_ms(500);
    PORTB = 0x00;
    _delay_ms(500);
  }
  return 0;
}
