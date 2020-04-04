/*
 *  yjsnpi.c
 *
 *  To run this project, set low FUSE bit 0b11100001,
 *  which means no CKDIV8, and use PLL for the system clock.
 *  The system clock will be 16MHz. 
 */
#define F_CPU (16000000UL)

#include <avr/io.h>
#include <avr/iotn85.h>
#include <avr/interrupt.h>
#include <util/delay_basic.h>

#define MAX_FIFO_COUNT (10)

struct MyFIFO_t {
  int size;
  int start;
  int end;
  uint16_t write_count;
  uint16_t read_count;
  uint8_t data[MAX_FIFO_COUNT];
} MyFIFO;

void fifo_init() {
  MyFIFO.size = MAX_FIFO_COUNT;
  MyFIFO.start = 0;
  MyFIFO.end = 0;
  MyFIFO.read_count = 0;
  MyFIFO.write_count = 0;
}

void fifo_write(uint8_t d) {
  // full
  if ((MyFIFO.write_count - MyFIFO.read_count) == MyFIFO.size)
    return;
  MyFIFO.data[MyFIFO.end] = d;
  MyFIFO.write_count++;
  MyFIFO.end = (MyFIFO.end + 1) % MyFIFO.size;
}

uint8_t fifo_read() {
  uint8_t d;
  // empty
  if ((MyFIFO.write_count - MyFIFO.read_count) == 0) {
    // return MyFIFO.data[(MyFIFO.size + MyFIFO.start - 1) % MyFIFO.size];
    return 128;
  }
  d = MyFIFO.data[MyFIFO.start];
  MyFIFO.read_count++;
  MyFIFO.start = (MyFIFO.start + 1) % MyFIFO.size;
  return d;
}

uint16_t const Fs = 14000;

/*
 * called every sampling cycle
 */
ISR(TIM0_COMPA_vect)
{
  OCR1B = fifo_read();
}

void i2c_clock_with_0(void) {
  PORTB &= ~(1<<DDB2); /* Clock LOW */
  _delay_loop_1(5); /* wait 35(7*5) CPU cycle -- Data input hold time */
  PORTB &= ~(1<<DDB0); /* SDA: 0 */
  _delay_loop_1(2); /* wait -- Data input setup time */
  PORTB |= (1<<DDB2); /* CLOCK HIGH */
  _delay_loop_1(7); /* Clock HIGH time */
}

void i2c_clock_with_1(void) {
  PORTB &= ~(1<<DDB2); /* Clock LOW */
  _delay_loop_1(5); /* wait 35(7*5) CPU cycle -- Data input hold time */
  PORTB |= (1<<DDB0); /* SDA: 1 */
  _delay_loop_1(2); /* wait -- Data input setup time */
  PORTB |= (1<<DDB2); /* CLOCK HIGH */
  _delay_loop_1(7); /* Clock HIGH time */
}

void i2c_clock_with_read_ack(void) {
  PORTB &= ~(1<<DDB2); /* Clock LOW */
  _delay_loop_1(7); /* wait 35(7*5) CPU cycle -- Data input hold time */
  PORTB |= (1<<DDB2); /* CLOCK HIGH */
  _delay_loop_1(5); /* Clock HIGH time */
  if (((1<<DDB0) & PORTB) == (1<<DDB0)) {
    PORTB |= (1<<DDB3); /* ERROR */
  }
  _delay_loop_1(2); /* Clock HIGH time */
}

uint8_t i2c_clock_with_read_1bit(void) {
  PORTB &= ~(1<<DDB2); /* Clock LOW */
  _delay_loop_1(7); /* wait 35(7*5) CPU cycle -- Data input hold time */
  PORTB |= (1<<DDB2); /* CLOCK HIGH */
  _delay_loop_1(7); /* Clock HIGH time */
  return (PORTB & (1<<DDB0)) ? 1 : 0;
}

int main(void)
{
  uint8_t chunk;

  /* set DDRB -- Data Direction Registor for port B
   * Set PB4, PB2, PB0, PB3 as OUTPUT
   * PB4: Sound output,
   * PB2: SCL,
   * PB0: SDA,
   * PB3: option
   */
  DDRB = (1<<DDB4)|(1<<DDB2)|(1<<DDB0)|(1<<DDB3);

  PORTB = 0x00;

  PLLCSR = (1<<PCKE)|(1<<PLLE); /* Select PLL clock for TC1.ck */
  OCR1C = 0xFF; /* TOP value */
  OCR1B = 0x00;
  GTCCR = (1<<PWM1B)|(1<<COM1B1)|(0<<COM1B0);
  TCCR1 = 0x01; /* Start TC1. CS13-10: 0001(PCK(64MHz) in asynchronous mode) */
  TCNT1 = 0;

  OCR0A  = (uint8_t)(F_CPU / 8 / Fs) - 1;
  TCCR0A = 0b00000010; /* Start TC0 as interval timer at 2MHz */
  TCCR0B = 0b00000010;

  TIMSK = (1<<OCIE0A);

  /*
   *  400kHz I2C Master
   */

  PORTB |= (1<<DDB2) | (1<<DDB0); /* Up SCL, SDA. Initial state. */
  _delay_loop_1(14); /* wait 42(14*3) CPU cycle */

  PORTB &= ~(1<<DDB0); /* To generate start state, down SDA. */
  _delay_loop_1(7); /* wait 21(7*3) CPU cycle -- 1312.5us */

  i2c_clock_with_1();
  i2c_clock_with_0();
  i2c_clock_with_1();
  i2c_clock_with_0();
  i2c_clock_with_0();
  i2c_clock_with_0();
  i2c_clock_with_0();

  i2c_clock_with_0(); /* W/R bit WRITE */

  i2c_clock_with_read_ack();

  i2c_clock_with_0();
  i2c_clock_with_0();
  i2c_clock_with_0();
  i2c_clock_with_0();
  i2c_clock_with_0();
  i2c_clock_with_0();
  i2c_clock_with_0();
  i2c_clock_with_0();

  i2c_clock_with_read_ack();
  
  i2c_clock_with_0();
  i2c_clock_with_0();
  i2c_clock_with_1();
  i2c_clock_with_0();
  i2c_clock_with_1();
  i2c_clock_with_1();
  i2c_clock_with_0();
  i2c_clock_with_0();

  i2c_clock_with_read_ack();

  PORTB &= ~(1<<DDB2); /* Clock LOW */
  _delay_loop_1(5); /* wait 35(7*5) CPU cycle -- Data input hold time */
  PORTB |= (1<<DDB0); /* SDA: 1 */
  _delay_loop_1(2); /* wait -- Data input setup time */
  PORTB |= (1<<DDB2); /* CLOCK HIGH */
  _delay_loop_1(7); /* Clock HIGH time */
  PORTB &= ~(1<<DDB0); /* To generate start state, down SDA. */
  _delay_loop_1(7); /* wait 21(7*3) CPU cycle -- 1312.5us */

  i2c_clock_with_1();
  i2c_clock_with_0();
  i2c_clock_with_1();
  i2c_clock_with_0();
  i2c_clock_with_0();
  i2c_clock_with_0();
  i2c_clock_with_0();

  i2c_clock_with_1(); /* W/R bit READ */

  i2c_clock_with_read_ack();

  sei();

  for (uint16_t i = 44; i < 63394 - 1; ++i) {
    chunk = 0b00000000;
    chunk |= i2c_clock_with_read_1bit() << 7;
    chunk |= i2c_clock_with_read_1bit() << 6;
    chunk |= i2c_clock_with_read_1bit() << 5;
    chunk |= i2c_clock_with_read_1bit() << 4;
    chunk |= i2c_clock_with_read_1bit() << 3;
    chunk |= i2c_clock_with_read_1bit() << 2;
    chunk |= i2c_clock_with_read_1bit() << 1;
    chunk |= i2c_clock_with_read_1bit() << 0;

    fifo_write(chunk);

    i2c_clock_with_0();
    
    while ((MyFIFO.write_count - MyFIFO.read_count) >= MyFIFO.size); /* Busy wait */
  }

  chunk = 0b00000000;
  chunk |= i2c_clock_with_read_1bit() << 7;
  chunk |= i2c_clock_with_read_1bit() << 6;
  chunk |= i2c_clock_with_read_1bit() << 5;
  chunk |= i2c_clock_with_read_1bit() << 4;
  chunk |= i2c_clock_with_read_1bit() << 3;
  chunk |= i2c_clock_with_read_1bit() << 2;
  chunk |= i2c_clock_with_read_1bit() << 1;
  chunk |= i2c_clock_with_read_1bit() << 0;
  
  i2c_clock_with_1();

  while ((MyFIFO.write_count - MyFIFO.read_count) != 0); /* Busy wait */

  PORTB &= ~(1<<DDB2); /* Clock LOW */
  _delay_loop_1(5); /* wait 35(7*5) CPU cycle -- Data input hold time */
  PORTB &= ~(1<<DDB0); /* SDA: 0 */
  _delay_loop_1(2); /* wait -- Data input setup time */
  PORTB |= (1<<DDB2); /* CLOCK HIGH */
  _delay_loop_1(7); /* Clock HIGH time */
  PORTB |= (1<<DDB0); /* To generate stop state, up SDA. */
  _delay_loop_1(7); /* wait 21(7*3) CPU cycle -- 1312.5us */

  PORTB &= ~(1<<DDB3);

  return 0;
}
