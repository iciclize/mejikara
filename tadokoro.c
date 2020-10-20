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
#include <avr/pgmspace.h>

FUSES = { .extended = 0xFF, .high = 0xDF, .low = 0xE1 };

#define SCL_LOW()   do { PORTB &= ~(1<<PB1); } while (0)
#define SCL_HIGH()  do { PORTB |= (1<<PB1); } while (0)
#define SDA_LOW()   do { PORTB &= ~(1<<PB0); } while (0)
#define SDA_HIGH()  do { PORTB |= (1<<PB0); } while (0)

#define I2C_HALF_CLOCK      (1.25)
#define I2C_FULL_CLOCK      (2.5)

/* MAX_FIFO_COUNT must be power of 2 (2^n) */
#define MAX_FIFO_COUNT (8)

uint8_t timer_div = 0;
volatile uint8_t div_count = 0;

volatile int fifoWp;
volatile int fifoRp;

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
    return fifoBuf[fifoRp];
  }
  fifoRp = next;
  return fifoBuf[fifoRp];
}

#define FIFO_ISFULL() (fifoRp == fifoWp)
#define FIFO_ISEMPTY() (((fifoRp+1)&(MAX_FIFO_COUNT-1)) == fifoWp)

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
  for (uint8_t mask = 0b10000000; mask > 0; mask >>= 1) {
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

const int8_t ima_index_table[16] __attribute__((__progmem__)) = {
  -1, -1, -1, -1, 2, 4, 6, 8,
  -1, -1, -1, -1, 2, 4, 6, 8
};

const uint16_t ima_step_table[89] __attribute__((__progmem__)) = {
  7, 8, 9, 10, 11, 12, 13, 14, 16, 17,
  19, 21, 23, 25, 28, 31, 34, 37, 41, 45,
  50, 55, 60, 66, 73, 80, 88, 97, 107, 118,
  130, 143, 157, 173, 190, 209, 230, 253, 279, 307,
  337, 371, 408, 449, 494, 544, 598, 658, 724, 796,
  876, 963, 1060, 1166, 1282, 1411, 1552, 1707, 1878, 2066,
  2272, 2499, 2749, 3024, 3327, 3660, 4026, 4428, 4871, 5358,
  5894, 6484, 7132, 7845, 8630, 9493, 10442, 11487, 12635, 13899,
  15289, 16818, 18500, 20350, 22385, 24623, 27086, 29794, 32767
};

struct IMA_WORK {
  uint8_t step_index;
  int16_t predictor;
};

uint16_t ima_decode(uint8_t nibble, struct IMA_WORK* work)
{
  uint16_t step;
  uint16_t diff;
  int32_t tmp;
  nibble &= 0b00001111;
  step = (uint16_t)pgm_read_word(&ima_step_table[work->step_index]);

  diff = step >> 3;
  if (nibble & 0b0100) diff += step;
  if (nibble & 0b0010) diff += (step >> 1);
  if (nibble & 0b0001) diff += (step >> 2);
  if (nibble & 0b1000) diff = -diff;

  tmp = work->predictor + diff;
  if (tmp < -32768) work->predictor = -32768;
  else if (tmp > 32767) work->predictor = 32767;
  else work->predictor = (int16_t)tmp;

  tmp = work->step_index + (int8_t)pgm_read_byte(&ima_index_table[nibble]);
  if (tmp < 0) work->step_index = 0;
  else if (tmp > 88) work->step_index = 88;
  else work->step_index = tmp;

  return work->predictor;
}

int main(void)
{
  uint16_t num_samples = 0;
  uint16_t block_size = 0;
  uint8_t adpcm = 0;
  uint8_t chunk;

  /* set DDRB -- Data Direction Registor for port B
   * Set PB4, PB3, PB1, PB0 as OUTPUT
   * PB4: Sound output,
   * PB3: Reversed sound output,
   * PB1: SCL,
   * PB0: SDA
   */
  DDRB = (1<<DDB4)|(1<<DDB1)|(1<<DDB0)|(1<<DDB3);
  PORTB = 0x00;

  FIFO_INIT();

  /*
   *  about <400kHz I2C Master
   */

  /*
   *  Read a WAV header. Start timers. Set a read addres.
   */
  {
    uint16_t Fs = 4545; /* sample rate */
    uint16_t p; /* an index where to read */

#define WAVE_SKIP_OFFSET ((uint16_t)12)

    /* 4 bytes in little endian -> uint32 */
#define SWAP_4(c1,c2,c3,c4) ( ((uint32_t)c4<<24) + ((uint32_t)c3<<16) \
    + ((uint16_t)c2<<8)+(uint8_t)c1)

    /*
     *   Set the read pointer to the position of the header (+skip_offset)
     */

    i2c_reset(); /* important */

    i2c_start();
    i2c_transmit(0b10100000); /* control byte (write) */
    i2c_transmit((uint8_t)(WAVE_SKIP_OFFSET >> 8)); /* high order address byte */
    i2c_transmit((uint8_t)WAVE_SKIP_OFFSET); /* low order address byte */
    i2c_start();
    i2c_transmit(0b10100001); /* control byte (read) */

    /* begin a block scope for a large buffer */
    {
      /*
       *  buffer 52 bytes, from the 12th byte to the 64th byte
       */
      uint8_t buf[52];
      uint32_t id, size;
      uint16_t tmp;

      for (uint8_t i = 0; i < 52; i++) {
        buf[i] = i2c_receive(0);
      }
      i2c_receive(1);
      i2c_stop();

     /* at the SKIP_OFFSETth byte (i.e. at the 12th, the head of 'fmt_').
      * omit RIFFxxxxWAVE */
      p = 0;
      do {
        id = *(uint32_t *)&buf[p]; /* chunk id  */
        size = *(uint32_t *)&buf[p+4]; /* size of data */
        p += 8; /* to the chunk data */
        switch (id) {
          case SWAP_4('f', 'm', 't', ' '):
            tmp = *(uint16_t *)&buf[p]; /* format id */
            switch (tmp) {
              case 0x0001: /* uncompressed linear PCM */
                adpcm = 0;
                break;
              case 0x0011: /* IMA-ADPCM */
                adpcm = 1;
                block_size = *(uint16_t *)&buf[p + 12];
                break;
              default:
                return 1;
            }
            tmp = *(uint16_t *)&buf[p+2]; /* the number of channels */
            if (tmp != 1) return 1; /* only 1 (mono) is accepted */
            Fs = *(uint32_t *)&buf[p+4]; /* sample rate */
            p += size; /* to the head of the next chunk */
            break;
          case SWAP_4('f', 'a', 'c', 't'):
            num_samples = *(uint32_t *)&buf[p];
            p += size; /* to the head of the next chunk */
            break;
          case SWAP_4('d', 'a', 't', 'a'):
            if (num_samples == 0) num_samples = size;
            /* stay at the data part of the chunk */
            break;
          default:
            return 1;
        }
      } while (id != SWAP_4('d', 'a', 't', 'a'));

    } /* end a scope for a buffer */

    /*
     *  Settings for Timer 1 (250kHz PWM carrier)
     */
    PLLCSR = (1<<PCKE)|(1<<PLLE); /* Select PLL clock for TC1.ck */
    OCR1C = 0xFF; /* TOP value. PWM resolution. Max PWM width. */
    OCR1B = 0x00; /* PWM Pulse width */
    /* enable PWM1B. Both PB4 and PB3 are output. PB3 is a reversed output. */
    GTCCR = (1<<PWM1B)|(0<<COM1B1)|(1<<COM1B0);
    TCCR1 = 0x01; /* Start TC1. CS[13:10] -- 0001(PCK(64MHz) in asynchronous mode) */
    TCNT1 = 0; /* Init the timer count */


    /*
     *  Settings for Timer 0 (sound data)
     */
    uint16_t ocrmax = (F_CPU / 8) / (Fs - 100);
    while (ocrmax > 256) {
      ocrmax = ocrmax / 2;
      timer_div++;
    }
    OCR0A  = (uint8_t)(ocrmax - 1);
    TCCR0A = (1<<WGM01)|(0<<WGM00);
    TCCR0B = (0<<WGM02)|(0<<CS02)|(1<<CS01)|(0<<CS00); /* clock select div8 */
    /* Start TC0 as interval timer at 2MHz */
    TIMSK = (1<<OCIE0A); /* Timer/Counter0 Output Compare Match A Interrupt Enable */


    /*
     *  Set the read pointer to the position of the data
     */
    i2c_start();
    i2c_transmit(0b10100000); /* control byte (write) */
    i2c_transmit((uint8_t)((p + WAVE_SKIP_OFFSET)>>8)); /* high order address byte */
    i2c_transmit((uint8_t)(p + WAVE_SKIP_OFFSET)); /* low order address byte */
    i2c_start();
    i2c_transmit(0b10100001); /* control byte (read) */

  }
  /*
   *  WAV header reading end
   */

  /*
   *  i2crom sequential read
   */
  if (adpcm == 1) {
    do {
      struct IMA_WORK ima_work;
      uint16_t samples_block; /* the rest of the samples in the current block */

      samples_block = (block_size - 4) * 2 + 1; /* wSamplesPerBlock */

      chunk = i2c_receive(0); /* IMA Samp0 LoByte */
      ima_work.predictor = (i2c_receive(0) << 8 | chunk); /* Hi:Lo */
      ima_work.step_index = i2c_receive(0); /* IMA Step Table Index */
      i2c_receive(0); /* IMA reserved byte */

      cli();
      fifo_write( ((ima_work.predictor^0x8000)+0x80) >> 8 );
      sei();

      do {
        uint8_t nibble[2]; /* 0: low nibble[3:0], 1: high nibble[7:4] */
        uint8_t ni;

        chunk = i2c_receive(0);
        nibble[0] = chunk;
        nibble[1] = chunk >> 4;

        for (ni = 0; ni < 2; ni++) {
          cli();
          fifo_write( ((ima_decode(nibble[ni], &ima_work)^0x8000)+0x80) >> 8 );
          sei();

          while (FIFO_ISFULL());
        }

        samples_block -= 2;
      } while (samples_block >= 2);

      num_samples -= (block_size - 4) * 2 + 1;
    } while (num_samples >= 4);
  } else {
    while (num_samples > 0) {
      chunk = i2c_receive(0);

      cli();
      fifo_write(chunk);
      sei();

      while (FIFO_ISFULL());

      num_samples--;
    }
  }

  i2c_receive(1);
  i2c_stop();

  while ( !FIFO_ISEMPTY() );

  GTCCR = (1<<PWM1B)|(0<<COM1B1)|(0<<COM1B0); /* detach OC1B(PB4,PB3) */
  TCCR1 = 0x00; /* Stop TC1. CS13-10: 0000 */
  PORTB |= (1<<DDB0) | (1<<DDB1) | (1<<DDB4) | (1<<DDB3);
  OCR1B = 0x00;

  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_mode();

  return 0;
}
