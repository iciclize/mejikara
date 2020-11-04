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
  .extended = 0xFF,
  .high = 0xDF,
  .low = 0xE2
};

#define FS_ADJUST (( +100 ))

#define SCL_LOW()   do { PORTB &= ~(1<<PB1); } while (0)
#define SCL_HIGH()  do { PORTB |= (1<<PB1); } while (0)
#define SDA_LOW()   do { PORTB &= ~(1<<PB0); } while (0)
#define SDA_HIGH()  do { PORTB |= (1<<PB0); } while (0)

#define I2C_HALF_CLOCK      (1.3)
#define I2C_HIGH_TIME       (0.8)

/*
 *  multiple sounds
 */
#define MAX_NUM_SOUND_TABLE (20)

uint8_t sound_index;
uint16_t sound_table[MAX_NUM_SOUND_TABLE];
volatile uint8_t play_irq = 0;

/* MAX_FIFO_COUNT must be power of 2 (2^n) */
#define MAX_FIFO_COUNT (8)

volatile int8_t fifoWp;
volatile int8_t fifoRp;

uint8_t fifoBuf[MAX_FIFO_COUNT];

#define FIFO_INIT()  do { fifoWp = 1; fifoRp = 0; } while (0)

void fifo_write(uint8_t v) {
  cli();
  if (fifoWp == fifoRp) {
    // full
    return;
  }
  fifoBuf[fifoWp] = v;
  /* optimization for (fifoWp + 1) % MAX_FIFO_COUNT */
  fifoWp = (fifoWp + 1) & (MAX_FIFO_COUNT - 1);
  sei();
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
 * be called every sample cycle
 */
ISR(TIM0_COMPA_vect)
{
  OCR1B = fifo_read() >> 2;
}

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
  int32_t diff;
  int32_t tmp;

  nibble &= 0b00001111;
  step = (uint16_t)pgm_read_word(&ima_step_table[work->step_index]);

  diff = step >> 3;
  if (nibble & 0b0100) diff += step;
  if (nibble & 0b0010) diff += (step >> 1);
  if (nibble & 0b0001) diff += (step >> 2);
  if (nibble & 0b1000) diff = -diff;

  tmp = work->predictor + diff;
  work->predictor = (int16_t)tmp;
  if (tmp < -32768) work->predictor = -32768;
  if (tmp > 32767) work->predictor = 32767;

  tmp = work->step_index + (int8_t)pgm_read_byte(&ima_index_table[nibble]);
  work->step_index = tmp;
  if (tmp < 0) work->step_index = 0;
  if (tmp > 88) work->step_index = 88;

  return work->predictor;
}

/* must be power of 2 */
#define RING_BUF_SIZE (16)

struct rom_ring_buffer {
  uint8_t i;
  uint8_t ring[RING_BUF_SIZE];
} rom_buf;

#define ROM_BUF_INIT() do { rom_buf.i = 0; } while (0)

void rom_seek_bytes(int8_t n) {
  // rom_buf.ri = (rom_buf.ri + n) % RING_BUF_SIZE;
  for (uint8_t i = 0; i < n; i++) {
    rom_buf.ring[rom_buf.i] = i2c_receive(0);
    rom_buf.i = (rom_buf.i + 1) & (RING_BUF_SIZE - 1);
  }
}

uint16_t rom_read_from_buffer_2(uint8_t offset) {
  uint16_t data;
  uint8_t pos;

  pos = (uint8_t)(rom_buf.i + offset) & (RING_BUF_SIZE - 1);
  data = rom_buf.ring[pos];
  pos = (pos + 1) & (RING_BUF_SIZE - 1);
  data |= rom_buf.ring[pos] << 8;
  
  return data;
}

void play(void)
{
  uint16_t num_samples = 0;
  uint16_t block_size = 0;
  uint8_t adpcm = 0;
  uint8_t chunk;

replay:
  play_irq = 0;

  if (play_irq)
    goto replay;

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

#define MAX_HEADER_READ_SIZE  (64)
#define WAVE_SKIP_OFFSET      (12)

  uint16_t ROM_READ_OFFSET =
    (uint16_t)sound_table[sound_index] + WAVE_SKIP_OFFSET;
  uint16_t offset_to_data = 0;

  /*
   *   Set the read pointer to the header + skip_offset
   */

  i2c_reset(); /* important */

  i2c_set_read_address(ROM_READ_OFFSET);

  /*
   *  buffer 16 bytes, from the 12th byte to the 28th byte
   */
  ROM_BUF_INIT();
  rom_seek_bytes(RING_BUF_SIZE);

  /* at the WAVE_SKIP_OFFSETth byte (i.e. at the 12th, the head of 'fmt_').
   * omit RIFFxxxxWAVE */
  while (1) {
    // uint32_t ckID, ckSize;
    // uint16_t wFormatTag;
    // uint16_t nChannels;
    // uint16_t nBlockAlign;
    // uint32_t nSamplesPerSec;
    // uint32_t dwSampleLength;
    uint16_t ckID, ckSize;
    uint16_t wFormatTag;
    uint16_t nChannels;
    uint16_t nBlockAlign;
    uint32_t nSamplesPerSec;
    uint32_t dwSampleLength;
    ckID = rom_read_from_buffer_2(0); /* chunk id  */
    /* Most Significant 2 bytes are always 0.
     * It's enough to read only Least Significant 2 bytes */
    ckSize = rom_read_from_buffer_2(4); /* chunk size */
    rom_seek_bytes(8); /* to the chunk data */
    offset_to_data += 8; 
    switch (ckID) {
      //case SWAP_4('f', 'm', 't', ' '):
      case SWAP_2('f', 'm'):
        wFormatTag = rom_read_from_buffer_2(0);
        switch (wFormatTag) {
          case 0x0001: /* uncompressed linear PCM */
            break;
          case 0x0011: /* IMA-ADPCM */
            adpcm = 1;
            nBlockAlign = rom_read_from_buffer_2(12);
            block_size = nBlockAlign;
            break;
          default:
            goto unsupported;
            break;
        }
        nChannels = rom_read_from_buffer_2(2);
        if (nChannels != 1) goto unsupported; /* only 1 (mono) is accepted */
        /* Fs must be within following
         * adpcm: 4010-17000Hz
         * lpcm:  4010-18000Hz */
        nSamplesPerSec = rom_read_from_buffer_2(4);
        Fs = nSamplesPerSec;

        rom_seek_bytes(ckSize);/* to the head of the next chunk */
        offset_to_data += ckSize; 
        break;
      //case SWAP_4('f', 'a', 'c', 't'):
      case SWAP_2('f', 'a'):
        dwSampleLength = rom_read_from_buffer_2(0);
        num_samples = dwSampleLength;

        rom_seek_bytes(ckSize); /* to the head of the next chunk */
        offset_to_data += ckSize;
        break;
      //case SWAP_4('d', 'a', 't', 'a'):
      case SWAP_2('d', 'a'):
        if (num_samples == 0)
          num_samples = ckSize;

        goto done_header;
        break;
      default:
        goto unsupported;
        break;
    }

    if ( offset_to_data < (MAX_HEADER_READ_SIZE - WAVE_SKIP_OFFSET) )
      continue;

    goto unsupported;
  }

done_header:
  /*
   *  Set the read pointer to the position of the data
   */
  i2c_end();

  i2c_set_read_address(ROM_READ_OFFSET + offset_to_data);

  /*
   *  Settings for Timer 1 (6bit 125kHz PWM carrier)
   */
  OCR1C = 63; /* TOP value. PWM resolution. Max PWM width. */

  /* enable PWM1B. Both PB4 and PB3 are output. PB3 is a reversed output. */
  GTCCR = (1<<PWM1B)|(0<<COM1B1)|(1<<COM1B0);

  TCCR1 = 0x01; /* Start TC1. CS[13:10] -- 0001(CPU Clock) */
  TCNT1 = 0; /* Init the timer count */


  /*
   *  Settings for Timer 0 (sound data)
   */
  uint16_t ocrmax = (F_CPU / 8) / (Fs + FS_ADJUST /* speed adjustment here */);
  OCR0A  = (uint8_t)(ocrmax - 1);
  TCCR0A = (1<<WGM01)|(0<<WGM00);
  TCCR0B = (0<<WGM02)|(0<<CS02)|(1<<CS01)|(0<<CS00); /* clock select div8 */

  /* Start TC0 as interval timer at 1MHz */
  TIMSK = (1<<OCIE0A); /* Timer/Counter0 Output Compare Match A */

  /*
   *  WAV header reading end
   */

  /*
   *  i2crom sequential read
   */
  if (adpcm) {
    do {
      struct IMA_WORK ima_work;
      uint16_t samples_block; /* the rest of the samples in the current block */
      const uint16_t wSamplesPerBlock
        = (block_size - 4) * 2 + 1;

      samples_block = wSamplesPerBlock;

      chunk = i2c_receive(0); /* IMA Samp0 LoByte */
      ima_work.predictor = (i2c_receive(0) << 8 | chunk); /* Hi:Lo */
      ima_work.step_index = i2c_receive(0); /* IMA Step Table Index */
      i2c_receive(0); /* IMA reserved byte */

      fifo_write((ima_work.predictor >> 8) + 0x80);

      do {
        uint8_t nibble0, nibble1; /* 0: low nibble[3:0], 1: high nibble[7:4] */

        chunk = i2c_receive(0);
        nibble0 = chunk;
        nibble1 = chunk >> 4;

        if (play_irq)
          goto replay;

        fifo_write((ima_decode(nibble0, &ima_work) >> 8) + 0x80);
        while (FIFO_ISFULL());

        fifo_write((ima_decode(nibble1, &ima_work) >> 8) + 0x80);
        while (FIFO_ISFULL());

        samples_block -= 2;
      } while (samples_block >= 2);

      num_samples -= wSamplesPerBlock;
    } while (num_samples >= 4);
  } else {
    while (num_samples > 0) {
      chunk = i2c_receive(0);

      fifo_write(chunk);

      if (play_irq)
        goto replay;

      while (FIFO_ISFULL());

      num_samples--;
    }
  }

  while ( !FIFO_ISEMPTY() );

unsupported:
  i2c_end();
  return;
}

ISR(INT0_vect)
{
  play_irq = 1;
}

int main(void) {
  PORTB |= (1<<PB2); /* INT0 pullup */
  MCUCR = (0<<ISC01)|(0<<ISC00);
  GIMSK = (1<<INT0);

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

  i2c_set_read_address(0);

  uint8_t NUM_SOUND_TABLE;

  NUM_SOUND_TABLE = i2c_receive(0);

  if (NUM_SOUND_TABLE == 'R') {
    /* Perhaps no header. 'R'IFF....WAVE */
    NUM_SOUND_TABLE = 1;
    sound_table[0] = 0;
  } else {
    i2c_receive(0); /* drop high byte */
    for (uint8_t i = 0; i < NUM_SOUND_TABLE; i++) {
      sound_table[i] = i2c_receive(0) | ((uint16_t)i2c_receive(0) << 8);
    }
  }
  i2c_end();

  sound_index = 0;

  while (1) {
    sound_index = sound_index % NUM_SOUND_TABLE;
    play();
    GTCCR = (1<<PWM1B)|(0<<COM1B1)|(0<<COM1B0); /* detach OC1B(PB4,PB3) */
    TCCR1 = 0x00; /* Stop TC1. CS13-10: 0000 */
    PORTB |= (1<<DDB0) | (1<<DDB1) | (1<<DDB4) | (1<<DDB3);
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    sleep_mode();
    sound_index++;
  }
}
