#include <stdint.h>
#include <stdio.h>

#define MAX_FIFO_COUNT 6

int fifoWp;
int fifoRp;
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
  fifoWp = (fifoWp + 1) % MAX_FIFO_COUNT;
}

uint8_t fifo_read() {
  int next;
  next = (fifoRp + 1) % MAX_FIFO_COUNT;
  if (next == fifoWp) { /* empty */
    return fifoBuf[fifoRp];
  }
  fifoRp = next;
  return fifoBuf[fifoRp];
}

int fifo_isFull() {
  return (fifoRp == fifoWp) ? 1 : 0;
}

int main(void) {
  fifo_init();

  printf("%d\n", fifo_read());
  printf("%d\n", fifo_read());
  fifo_write(1);
  fifo_write(2);
  fifo_write(3);
  fifo_write(4);
  fifo_write(5);
  fifo_write(6);
  fifo_write(7);
  fifo_write(8);
  fifo_write(9);
  printf("%d\n", fifo_read());
  printf("%d\n", fifo_read());
  printf("%d\n", fifo_read());
  printf("%d\n", fifo_read());
  printf("%d\n", fifo_read());
  printf("%d\n", fifo_read());
  printf("%d\n", fifo_read());
  printf("%d\n", fifo_read());
  fifo_write(10);
  fifo_write(11);
  printf("%d\n", fifo_read());
  printf("%d\n", fifo_read());
  printf("%d\n", fifo_read());
  printf("%d\n", fifo_read());
  printf("%d\n", fifo_read());

  return 0;
}
