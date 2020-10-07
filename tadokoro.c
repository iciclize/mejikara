/*
 *  tadokoro.c
 *
 *  To run this project, set low FUSE bit 0b11100001,
 *  which means CKDIV8 is disabled and PLL is enabled for the system clock.
 *  The system clock will be 16MHz. 
 */
#define F_CPU (16000000UL)

#include <avr/io.h>
#ifndef YJSNPI_MAKE
  #include <avr/iotn85.h> /* for autocompletion */
#endif
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <util/delay.h>

FUSES = { .extended = 0xFF, .high = 0xDF, .low = 0xE1 };

#define SCL_LOW()   do { PORTB &= ~(1<<PB1); } while (0)
#define SCL_HIGH()  do { PORTB |= (1<<PB1); } while (0)
#define SDA_LOW()   do { PORTB &= ~(1<<PB0); } while (0)
#define SDA_HIGH()  do { PORTB |= (1<<PB0); } while (0)

#define I2C_HALF_CLOCK      (1.25)
#define I2C_FULL_CLOCK      (2.5)

/* MAX_FIFO_COUNT must be power of 2 (2^n) */
#define MAX_FIFO_COUNT (8)

#define SAMPLING_FREQUENCY (22000 - 100)
#define WAV_FILE_SIZE      (22062)

uint16_t Fs = SAMPLING_FREQUENCY;
uint16_t num_samples = 0;
uint8_t adpcm = 0;
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
 * be called every sample cycle
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

void i2c_start(void) {
  SDA_HIGH();
  _delay_us(I2C_HALF_CLOCK);
  SCL_HIGH();
  _delay_us(I2C_HALF_CLOCK);
  SDA_LOW();
  _delay_us(I2C_HALF_CLOCK);
  SCL_LOW();
}

void i2c_stop(void) {
  SDA_LOW();
  _delay_us(I2C_HALF_CLOCK);
  SCL_HIGH();
  _delay_us(I2C_HALF_CLOCK);
  SDA_HIGH();
}

void i2c_reset(void) {
  SDA_HIGH();
  _delay_us(I2C_HALF_CLOCK);

  for (uint8_t i = 0; i < 8; i++) {
    SCL_HIGH(); _delay_us(I2C_HALF_CLOCK);
    SCL_LOW(); _delay_us(I2C_HALF_CLOCK);
  }
  SCL_HIGH(); _delay_us(I2C_HALF_CLOCK);

  /* generate start condition */
  SDA_LOW(); _delay_us(I2C_HALF_CLOCK);

  /* generate stop condition */
  SCL_HIGH(); _delay_us(I2C_HALF_CLOCK);
  SDA_HIGH(); _delay_us(I2C_HALF_CLOCK);

  /* down bus */
  SCL_LOW(); _delay_us(I2C_HALF_CLOCK);
  SDA_LOW(); _delay_us(I2C_HALF_CLOCK);
}

uint8_t i2c_transmit(uint8_t data) {
  uint8_t nack = 0;
  for (uint16_t mask = 1<<7; mask > 0; mask >>= 1) {
    if ((data & mask) != 0) {
      SDA_HIGH();
    } else {
      SDA_LOW();
    }
    _delay_us(I2C_HALF_CLOCK);
    SCL_HIGH(); _delay_us(I2C_HALF_CLOCK);
    SCL_LOW(); _delay_us(I2C_HALF_CLOCK);
  }
  /* Receive ACK */
  SDA_HIGH();
  DDRB &= ~(1<<DDB0); /* SDA as input. Pull-up is also enabled. */
  _delay_us(I2C_HALF_CLOCK);
  SCL_HIGH();
  if ( ((1<<PINB0) & PINB) != 0) {
    /* NACK */
    nack = 1;
  }
  _delay_us(I2C_HALF_CLOCK);
  SCL_LOW();
  DDRB |= (1<<DDB0); /* SDA as output. */
  return nack;
}

uint8_t i2c_receive(uint8_t nack) {
  uint8_t buf = 0b00000000;
  SDA_HIGH();
  DDRB &= ~(1<<DDB0); /* SDA as input. Pull-up is also enabled. */

  for (uint8_t i = 0; i < 8; i++) {
    buf <<= 1;
    SCL_HIGH();
    _delay_us(I2C_HALF_CLOCK);
    if ( ((1<<PINB0) & PINB) != 0) {
      buf += 1;
    }
    SCL_LOW();
    _delay_us(I2C_HALF_CLOCK);
  }

  /* Send ACK */
  DDRB |= (1<<DDB0); /* SDA as output. */
  if (nack) {
    SDA_HIGH();
  } else {
    SDA_LOW();
  }
  _delay_us(I2C_HALF_CLOCK);
  SCL_HIGH(); _delay_us(I2C_HALF_CLOCK);
  SCL_LOW(); _delay_us(I2C_HALF_CLOCK);
  return buf;
}

int main(void)
{
  uint8_t chunk;

  /* set DDRB -- Data Direction Registor for port B
   * Set PB4, PB1, PB0, PB3 as OUTPUT
   * PB4: Sound output,
   * PB1: SCL,
   * PB0: SDA,
   * PB3: option
   */
  DDRB = (1<<DDB4)|(1<<DDB1)|(1<<DDB0)|(1<<DDB3);
  PORTB = 0x00;

  /*
   *  about 200kHz I2C Master
   */
  uint8_t buf[44];
  uint32_t id, size;
  uint16_t tmp;
  uint8_t p;

  /* 4 bytes in little endian -> uint32 */
  #define SWAP_4(c1,c2,c3,c4) (((uint32_t)c4<<24)+((uint32_t)c3<<16)+((uint16_t)c2<<8)+(uint8_t)c1)
  /* 2 bytes in little endian -> uint16 */
  #define SWAP_2(c1,c2) (((uint16_t)c2<<8)+(uint8_t)c1)

#define CONCAT_4(c1,c2,c3,c4) (((uint32_t)c1<<24)+((uint32_t)c2<<16)+((uint16_t)c3<<8)+(uint8_t)c4)

  cli();
  i2c_reset();

  i2c_start();
  i2c_transmit(0b10100000); /* control byte (write) */
  i2c_transmit(0x00); /* high order address byte */
  i2c_transmit(0x00); /* low order address byte */
  i2c_start();
  i2c_transmit(0b10100001); /* control byte (read) */
  for (uint8_t i = 0; i < 44; i++) {
    buf[i] = i2c_receive(0);
  }
  i2c_receive(1);
  i2c_stop();

  p = 12;
  do {
    id = CONCAT_4(buf[p], buf[p+1], buf[p+2], buf[p+3]); /* chunk id  */
    size = SWAP_4(buf[p+4], buf[p+5], buf[p+6], buf[p+7]); /* size of data */
    p += 8; /* to chunk data */
    switch (id) {
      case CONCAT_4('f', 'm', 't', ' '):
        tmp = SWAP_2(buf[p], buf[p+1]); /* format id */
        switch (tmp) {
          case 0x0001: /* non-compressed linear PCM */
            adpcm = 0;
            break;
          case 0x0011: /* IMA-ADPCM */
            adpcm = 1;
            break;
          default:
            return 1;
        }
        break;
        tmp = SWAP_2(buf[p+2], buf[p+3]); /* number of channels (must be mono) */
        if (tmp != 0) return 1;
        Fs = SWAP_4(buf[p+4], buf[p+5], buf[p+6], buf[p+7]); /* sample rate */
        p += size; /* to next chunk head */
        break;
      case CONCAT_4('f', 'a', 'c', 't'):
        num_samples = SWAP_4(buf[p], buf[p+1], buf[p+2], buf[p+3]);
        p += size; /* to next chunk head */
        break;
      case CONCAT_4('d', 'a', 't', 'a'):
        if (num_samples == 0) num_samples = size;
        /* stay at chunk data */
        break;
      default:
        return 1;
    }
  } while (id != CONCAT_4('d', 'a', 't', 'a'));

  PLLCSR = (1<<PCKE)|(1<<PLLE); /* Select PLL clock for TC1.ck */
  OCR1C = 0xFF; /* TOP value. PWM resolution. Max PWM width. */
  OCR1B = 0x00; /* PWM Pulse width */
  /* enable PWM1B. Both PB4 and PB3 are output */
  GTCCR = (1<<PWM1B)|(0<<COM1B1)|(1<<COM1B0);
  TCCR1 = 0x01; /* Start TC1. CS[13:10] -- 0001(PCK(64MHz) in asynchronous mode) */
  TCNT1 = 0;

  Fs = Fs - 100;

  uint16_t ocrmax = (F_CPU / 8) / Fs;
  while (ocrmax > 256) {
    ocrmax = ocrmax / 2;
    timer_div++;
  }
  OCR0A  = (uint8_t)(ocrmax - 1);
  /* Start TC0 as interval timer at 2MHz */
  TCCR0A = (0<<WGM02)|(1<<WGM01)|(0<<WGM00);
  TCCR0B = (0<<CS02)|(1<<CS01)|(0<<CS00); /* clock select div8 */

  TIMSK = (1<<OCIE0A); /* Timer/Counter0 Output Compare Match A Interrupt Enable */

  i2c_start();
  i2c_transmit(0b10100000); /* control byte (write) */
  i2c_transmit(p>>8); /* high order address byte */
  i2c_transmit(p); /* low order address byte */
  i2c_start();
  i2c_transmit(0b10100001); /* control byte (read) */

  do {
    chunk = i2c_receive(0);

    /* fifo_isFull() */
    while (fifoRp == fifoWp);

    cli();
    fifo_write(chunk);
    sei();

    num_samples--;
  } while (num_samples > 0);

  i2c_receive(1);
  
  i2c_stop();

  GTCCR = (1<<PWM1B)|(0<<COM1B1)|(0<<COM1B0); /* detach OC1B(PB4,3) */
  TCCR1 = 0x00; /* Stop TC1. CS13-10: 0000 */
  PORTB |= (1<<DDB0) | (1<<DDB1) | (1<<DDB4) | (1<<DDB3);
  OCR1B = 0x00;

  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_mode();

  return 0;
}
