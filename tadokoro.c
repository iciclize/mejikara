/*
 *
 *  tadokoro.c
 *
 */
#define F_CPU (8000000UL)

#include <avr/io.h>
#ifndef YJSNPI_MAKE
  #include <avr/iotn85.h> /* for autocompletion */
#endif
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <util/delay.h>
#include <avr/pgmspace.h>

FUSES = {
  .low = 0xE2,
  .high = 0xDF,
  .extended = 0xFF
};

/*
 *  Software i2c routines
 */

#define SCL_LOW()   do { PORTB &= ~(1<<PB1); } while (0)
#define SCL_HIGH()  do { PORTB |= (1<<PB1); } while (0)
#define SDA_LOW()   do { PORTB &= ~(1<<PB0); } while (0)
#define SDA_HIGH()  do { PORTB |= (1<<PB0); } while (0)

#define I2C_HALF_CLOCK      (1.3)
#define I2C_HIGH_TIME       (0.8)

// prototype declaration
void i2c_start(void);
void i2c_stop(void);
void i2c_reset(void);
uint8_t i2c_transmit(uint8_t data);
uint8_t i2c_receive(uint8_t nack);
void i2c_set_read_address(uint16_t read_address);
void i2c_end(void);


/*
 * Sound buffer
 */

// MAX_FIFO_COUNT must be power of 2 (2^n) for mod optimization
#define MAX_FIFO_COUNT (8)

volatile int8_t fifoWp;
volatile int8_t fifoRp;

uint8_t fifoBuf[MAX_FIFO_COUNT];

#define FIFO_INIT()  do { fifoWp = 1; fifoRp = 0; } while (0)

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
    // return 0xFF;
    return fifoBuf[fifoRp];
  }
  fifoRp = next;
  return fifoBuf[fifoRp];
}

#define FIFO_ISFULL() (fifoRp == fifoWp)
#define FIFO_ISEMPTY() (((fifoRp+1)&(MAX_FIFO_COUNT-1)) == fifoWp)


/*
 * Button input detection with chattering elimination
 */

uint16_t detect_countdown = 0;
uint8_t pressed = 0;


/*
 * Read/Seek for reading a header
 */

uint32_t read_rom_4() {
  uint32_t data = 0;
  for (uint8_t i = 0; i < 4; i++)
    data |= (uint32_t)i2c_receive(0) << (8 * i);
  return data;
}

uint16_t read_rom_2() {
  uint16_t data = 0;
  for (uint8_t i = 0; i < 2; i++)
    data |= (uint16_t)i2c_receive(0) << (8 * i);
  return data;
}

void seek_rom_bytes(uint8_t n) {
  for (uint8_t i = 0; i < n; i++)
    i2c_receive(0);
}


/*
 * Sound streamer
 *
 * This is called every sample cycle.
 */

ISR(TIM0_COMPA_vect)
{
  OCR1B = fifo_read() >> 2;
}


/*
 * Controller
 */

void play(void)
{
  uint16_t num_samples = 0;
  uint8_t chunk;

replay:
  PORTB &= ~((1<<PB4)|(1<<PB3)|(1<<PB1)|(1<<PB0));

  FIFO_INIT();

  /*
   *  about <400kHz I2C Master
   */

  /*
   *  Read a WAV header. Start timers. Set a read addres.
   */
  uint16_t Fs = 4545; /* sample rate */

/* 4 bytes in little endian -> uint32 */
#define SWAP_4(c1,c2,c3,c4) ( ((uint32_t)c4<<24) + ((uint32_t)c3<<16) \
  + ((uint16_t)c2<<8)+(uint8_t)c1)
#define SWAP_2(c1,c2) ( ((uint16_t)c2<<8) + (uint8_t)c1 )

#define RIFF_WAVE_POSITION    (0)

  //
  // Set the read pointer to the head of RIFF????WAVE
  //

  // important
  i2c_reset();
  i2c_set_read_address(RIFF_WAVE_POSITION);

  // read 0-3 bytes of the ROM
  if (read_rom_4() != SWAP_4('R', 'I', 'F', 'F')) {
    goto unsupported;
  }

  // skip 4-7 bytes of the ROM
  seek_rom_bytes(4);

  // read 8-11 bytes of the ROM
  if (read_rom_4() != SWAP_4('W', 'A', 'V', 'E')) {
    goto unsupported;
  }

  // now at the "fmt " chunk

  uint8_t content_address = 0;

  while (1) {
    uint32_t ckID, ckSize;
    uint16_t wFormatTag;
    uint16_t nChannels;
    uint32_t nSamplesPerSec;

    // Now at the head of every chunk
    uint8_t chunk_remaining_size;

    ckID = read_rom_4(); /* chunk id  */
    ckSize = read_rom_4(); /* chunk size */

    chunk_remaining_size = ckSize;
    content_address += 8;

    switch (ckID) {
      case SWAP_4('f', 'm', 't', ' '):

        wFormatTag = read_rom_2();
        chunk_remaining_size -= 2;
        
        switch (wFormatTag) {
          case 0x0001: /* uncompressed linear PCM */
            break;
          case 0x0011: /* IMA-ADPCM */
          default:
            goto unsupported;
            break;
        }

        nChannels = read_rom_2();
        chunk_remaining_size -= 2;

        // only 1 (mono) is accepted
        if (nChannels != 1)
          goto unsupported;

        nSamplesPerSec = read_rom_4();
        chunk_remaining_size -= 4;

        // Fs must be within below
        // adpcm: 4010-17000Hz
        // lpcm:  4010-18000Hz
        Fs = nSamplesPerSec;

        // to the head of the next chunk
        seek_rom_bytes(chunk_remaining_size);

        content_address += ckSize;
        break;

      case SWAP_4('d', 'a', 't', 'a'):
        num_samples = ckSize;
        goto done_header;
        break;

      default:
        goto unsupported;
        break;
    }
  }

done_header:
  // Set the read pointer to the position of the data
  i2c_end();
  i2c_set_read_address(content_address);

  //
  // Settings for Timer 1 (6bit 125kHz PWM carrier)
  //

  // TOP value. PWM resolution. Max PWM width.
  OCR1C = 63;

  // enable PWM1B. Both PB4 and PB3 are output. PB3 is a reversed output.
  GTCCR = (1<<PWM1B)|(0<<COM1B1)|(1<<COM1B0);

  // Start TC1. CS[13:10] -- 0001(CPU Clock)
  TCCR1 = 0x01;

  // Init the timer count
  TCNT1 = 0;


  //
  // Settings for Timer 0 (sound data)
  //

  uint16_t ocrmax = (F_CPU / 8) / (Fs - 0/* speed adjustment here */);
  OCR0A  = (uint8_t)(ocrmax);
  TCCR0A = (1<<WGM01)|(0<<WGM00);
  TCCR0B = (0<<WGM02)|(0<<CS02)|(1<<CS01)|(0<<CS00); /* clock select div8 */

  //
  // Start TC0 as interval timer at 1MHz */
  //

  // Timer/Counter0 Output Compare Match A
  TIMSK = (1<<OCIE0A);

  /*
   *  WAV header reading end
   */

  /*
   *  i2crom sequential read
   */
  while (num_samples > 0) {
    chunk = i2c_receive(0);

    cli();
    fifo_write(chunk);
    sei();

    if (detect_countdown >= 2) {
      detect_countdown--;
    } else if (detect_countdown == 1) {
      detect_countdown--;
      if (pressed == 0) {
        if (bit_is_clear(PINB, PINB2)) {
          pressed = 1;
          goto replay;
        }
      } else {
        if (bit_is_set(PINB, PINB2)) {
          pressed = 0;
        }
      }
    } else if (detect_countdown == 0) {
      if (pressed == 0) {
        if (bit_is_clear(PINB, PINB2))
          detect_countdown = 300;
      } else {
        if (bit_is_set(PINB, PINB2))
          detect_countdown = 300;
      }
    }

    while (FIFO_ISFULL());

    num_samples--;
  }

  while ( !FIFO_ISEMPTY() );

unsupported:
  i2c_end();
  return;
}

EMPTY_INTERRUPT(INT0_vect);

/*
 * Wake up and sleep
 */

int main(void) {
  PORTB |= (1<<PB2); /* INT0 pullup */

  /* set DDRB -- Data Direction Registor for port B
   * Set PB4, PB3, PB1, PB0 as OUTPUT
   * PB4: Sound output,
   * PB3: Reversed sound output,
   * PB1: SCL,
   * PB0: SDA
   */
  DDRB = (1<<DDB4)|(1<<DDB3)|(1<<DDB1)|(1<<DDB0);
  PORTB &= ~((1<<PB4)|(1<<PB3)|(1<<PB1)|(1<<PB0));

  i2c_reset(); /* important */

  while (1) {
    GTCCR = (1<<PWM1B)|(0<<COM1B1)|(0<<COM1B0); /* detach OC1B(PB4,PB3) */
    TCCR1 = 0x00; /* Stop TC1. CS13-10: 0000 */
    PORTB |= (1<<DDB0) | (1<<DDB1) | (1<<DDB4) | (1<<DDB3);

    cli();
    MCUCR = (0<<ISC01)|(0<<ISC00); /* wake up by low level */
    GIMSK = (1<<INT0); /* enable interrupt */

    set_sleep_mode(SLEEP_MODE_PWR_DOWN);

    /* sleep sequence from the avr-libc's home page */
    sleep_enable();
    sleep_bod_disable();
    sei();
    sleep_cpu();
    /* now woken up */
    sleep_disable();

    GIMSK = (0<<INT0); /* disable interrupt */

    play();
  }
}


/*
 * I2C routines
 */

void i2c_start(void) {
  SDA_HIGH();
  _delay_us(I2C_HALF_CLOCK);
  SCL_HIGH();
  _delay_us(I2C_HIGH_TIME);
  SDA_LOW();
  _delay_us(I2C_HALF_CLOCK);
  SCL_LOW();
}

void i2c_stop(void) {
  SDA_LOW();
  _delay_us(I2C_HALF_CLOCK);
  SCL_HIGH();
  _delay_us(I2C_HIGH_TIME);
  SDA_HIGH();
}

void i2c_reset(void) {
  SDA_HIGH();
  _delay_us(I2C_HALF_CLOCK);

  for (uint8_t i = 0; i < 8; i++) {
    SCL_HIGH(); _delay_us(I2C_HIGH_TIME);
    SCL_LOW(); _delay_us(I2C_HALF_CLOCK);
  }
  SCL_HIGH(); _delay_us(I2C_HIGH_TIME);

  /* generate start condition */
  SDA_LOW(); _delay_us(I2C_HALF_CLOCK);

  /* generate stop condition */
  SCL_HIGH(); _delay_us(I2C_HIGH_TIME);
  SDA_HIGH(); _delay_us(I2C_HALF_CLOCK);

  /* down bus */
  SCL_LOW(); _delay_us(I2C_HALF_CLOCK);
  SDA_LOW(); _delay_us(I2C_HALF_CLOCK);
}

uint8_t i2c_transmit(uint8_t data) {
  uint8_t nack = 0;
  for (uint8_t mask = 0b10000000; mask > 0; mask >>= 1) {
    if ((data & mask) != 0) {
      SDA_HIGH();
    } else {
      SDA_LOW();
    }
    _delay_us(I2C_HALF_CLOCK);
    SCL_HIGH(); _delay_us(I2C_HIGH_TIME);
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
  _delay_us(I2C_HIGH_TIME);
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
    _delay_us(I2C_HIGH_TIME);
    if ( ((1<<PINB0) & PINB) != 0) {
      buf |= 1;
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
  SCL_HIGH(); _delay_us(I2C_HIGH_TIME);
  SCL_LOW(); _delay_us(I2C_HALF_CLOCK);
  return buf;
}

void i2c_set_read_address(uint16_t read_address) {
  i2c_start();
  i2c_transmit(0b10100000); /* control byte (write) */
  i2c_transmit((uint8_t)(read_address >> 8)); /* high order address byte */
  i2c_transmit((uint8_t)read_address); /* low order address byte */
  i2c_start();
  i2c_transmit(0b10100001); /* control byte (read) */
}

void i2c_end(void) {
  i2c_receive(1);
  i2c_stop();
}
