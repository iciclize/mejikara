#include <avr/pgmspace.h>

#define NUM_SOUND_TABLE (9)
const uint16_t sound_table[NUM_SOUND_TABLE] __attribute__((__progmem__)) =
{0,
5180,
10490,
16312,
28534,
33332,
36594,
39856,
43630};
