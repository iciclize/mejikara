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
#include <avr/sleep.h>
#include <util/delay.h>

#define HALF_CLOCK          (1.25)
#define FULL_CLOCK          (2.5)

#define WAIT_FOR_CHANGE_SDA (0.75)
#define WAIT_FOR_SCL_UP     (0.5)

#define WAIT_FOR_RISETIME   (0.5)
#define WAIT_FOR_SCL_DOWN   (0.75)

/* MAX_FIFO_COUNT must be power of 2 (2^n) */
#define MAX_FIFO_COUNT (8)

#define SAMPLING_FREQUENCY (6200)
#define WAV_FILE_SIZE      (61573)

unsigned long Fs = SAMPLING_FREQUENCY;
uint8_t timer_div = 0;
volatile uint8_t div_count = 0;

volatile int fifoWp;
volatile int fifoRp;

uint8_t fifoBuf[MAX_FIFO_COUNT];

void fifo_init() {
  fifoWp = 1;
  fifoRp = 0;
}

void fifo_write(uint8_t v) {
  if (fifoWp == fifoRp) {
    // full
    return;
  }
  fifoBuf[fifoWp] = v;
  /* optimization for (fifoWp + 1) % MAX_FIFO_COUNT */
  fifoWp = (fifoWp + 1) & (MAX_FIFO_COUNT - 1);
}

uint8_t fifo_read() {
  int next;
  next = (fifoRp + 1) & (MAX_FIFO_COUNT - 1);
  if (next == fifoWp) { /* empty */
    return fifoBuf[fifoRp];
  }
  fifoRp = next;
  return fifoBuf[fifoRp];
}

int fifo_isFull() {
  return (fifoRp == fifoWp) ? 1 : 0;
}

/*
 * called every sampling cycle
 */
ISR(TIM0_COMPA_vect)
{
  if (div_count < timer_div) {
    div_count++;
  } else {
    OCR1B = fifo_read();
    div_count = 0;
  }
}

void i2c_clock_with_0(void) {
  PORTB &= ~(1<<DDB2); /* Clock LOW */
  DDRB |= (1<<DDB0); /* OUTPUT */
  _delay_us(WAIT_FOR_CHANGE_SDA); /* wait 15(5*3) CPU cycle -- Data input hold time */
  PORTB &= ~(1<<DDB0); /* SDA: 0 */
  _delay_us(WAIT_FOR_SCL_UP); /* wait -- Data input setup time */
  PORTB |= (1<<DDB2); /* CLOCK HIGH */
  _delay_us(HALF_CLOCK); /* Clock HIGH time */
}

void i2c_clock_with_1(void) {
  PORTB &= ~(1<<DDB2); /* Clock LOW */
  DDRB |= (1<<DDB0); /* OUTPUT */
  _delay_us(WAIT_FOR_CHANGE_SDA); /* wait 15(5*3) CPU cycle -- Data input hold time */
  PORTB |= (1<<DDB0); /* SDA: 1 */
  _delay_us(WAIT_FOR_SCL_UP); /* wait -- Data input setup time */
  PORTB |= (1<<DDB2); /* CLOCK HIGH */
  _delay_us(HALF_CLOCK); /* Clock HIGH time */
}

void i2c_clock_with_read_ack(void) {
  PORTB &= ~(1<<DDB2); /* Clock LOW */
  DDRB &= ~(1<<DDB0); /* INPUT */
  PORTB |= (1<<DDB0); /* ENABLE PULLUP */
  _delay_us(HALF_CLOCK); /* wait 21(7*3) CPU cycle */
  PORTB |= (1<<DDB2); /* CLOCK HIGH */
  _delay_us(WAIT_FOR_RISETIME);
  if ( ((1<<PINB2) & PINB) == 0) {
    PORTB |= (1<DDB3); /* ERROR: SCL LOW */
  }
  if ( ((1<<PINB0) & PINB) != 0) {
    PORTB |= (1<<DDB3); /* ERROR: NO ACK */
  }
  _delay_us(WAIT_FOR_SCL_DOWN); /* Clock HIGH time */
}

uint8_t i2c_clock_with_read_1bit(void) {
  uint8_t bit;
  PORTB &= ~(1<<DDB2); /* Clock LOW */
  DDRB &= ~(1<<DDB0); /* INPUT */
  PORTB |= (1<<DDB0); /* ENABLE PULLUP */
  _delay_us(HALF_CLOCK); /* wait 21(7*3) CPU cycle -- Data input hold time */
  PORTB |= (1<<DDB2); /* CLOCK HIGH */
  _delay_us(WAIT_FOR_RISETIME); /* wait pullup */
  if ( ((1<<PINB2) & PINB) != (1<<PINB2)) {
    PORTB |= (1<<DDB3); /* ERROR */
  }
  bit = ((PINB & (1<<PINB0)) != 0) ? 1 : 0;
  _delay_us(WAIT_FOR_SCL_DOWN);
  return bit;
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

  unsigned long ocrmax = (F_CPU / 8) / Fs;
  while (ocrmax > 256) {
    ocrmax = ocrmax / 2;
    timer_div++;
  }
  OCR0A  = (uint8_t)(ocrmax - 1);
  TCCR0A = 0b00000010; /* Start TC0 as interval timer at 2MHz */
  TCCR0B = 0b00000010;

  TIMSK = (1<<OCIE0A);

  /*
   *  400kHz I2C Master
   */

  cli();

  /* stop condition start */
  PORTB &= ~(1<<DDB2); /* Clock LOW */
  DDRB |= (1<<DDB0); /* OUTPUT */
  _delay_us(WAIT_FOR_CHANGE_SDA); /* wait 35(7*5) CPU cycle -- Data input hold time */
  PORTB &= ~(1<<DDB0); /* SDA: 0 */
  _delay_us(WAIT_FOR_SCL_UP); /* wait -- Data input setup time */
  PORTB |= (1<<DDB2); /* CLOCK HIGH */
  _delay_us(FULL_CLOCK); /* Clock HIGH time */
  PORTB |= (1<<DDB0); /* To generate stop state, up SDA. */
  _delay_us(FULL_CLOCK); /* wait 21(7*3) CPU cycle -- 1312.5us */
  /* stop condition end */


  PORTB |= (1<<DDB2) | (1<<DDB0); /* Up SCL, SDA. Initial state. */
  _delay_us(FULL_CLOCK); /* wait 42(14*3) CPU cycle */

  PORTB &= ~(1<<DDB0); /* To generate start state, down SDA. */
  _delay_us(FULL_CLOCK); /* wait 42(14*3) CPU cycle -- 1312.5us */

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

  /* Generate start condition */
  PORTB &= ~(1<<DDB2); /* Clock LOW */
  DDRB |= (1<<DDB0); /* OUTPUT */
  _delay_us(WAIT_FOR_CHANGE_SDA); /* wait 35(7*5) CPU cycle -- Data input hold time */
  PORTB |= (1<<DDB0); /* SDA: 1 */
  _delay_us(WAIT_FOR_SCL_UP); /* wait -- Data input setup time */
  PORTB |= (1<<DDB2); /* CLOCK HIGH */
  _delay_us(FULL_CLOCK); /* Clock HIGH time */
  PORTB &= ~(1<<DDB0); /* To generate start state, down SDA. */
  _delay_us(FULL_CLOCK); /* wait 42(14*3) CPU cycle -- 2625us */

  i2c_clock_with_1();
  i2c_clock_with_0();
  i2c_clock_with_1();
  i2c_clock_with_0();
  i2c_clock_with_0();
  i2c_clock_with_0();
  i2c_clock_with_0();

  i2c_clock_with_1(); /* W/R bit READ */

  i2c_clock_with_read_ack();

  /* 63394 bytes (header 44 bytes) */
  for (uint16_t i = 44; i < WAV_FILE_SIZE - 1; ++i) {
    cli();
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

    sei();

    /* fifo_isFull() */
    while (fifoRp == fifoWp);

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

  PORTB &= ~(1<<DDB2); /* Clock LOW */
  DDRB |= (1<<DDB0); /* OUTPUT */
  _delay_us(WAIT_FOR_CHANGE_SDA); /* wait 35(7*5) CPU cycle -- Data input hold time */
  PORTB &= ~(1<<DDB0); /* SDA: 0 */
  _delay_us(WAIT_FOR_SCL_UP); /* wait -- Data input setup time */
  PORTB |= (1<<DDB2); /* CLOCK HIGH */
  _delay_us(FULL_CLOCK); /* Clock HIGH time */
  PORTB |= (1<<DDB0); /* To generate stop state, up SDA. */
  _delay_us(FULL_CLOCK); /* wait 21(7*3) CPU cycle -- 1312.5us */

  PORTB &= ~((1<<DDB0) | (1<<DDB2) | (1<<DDB4) | (1<<DDB3));

  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_mode();

  return 0;
}
