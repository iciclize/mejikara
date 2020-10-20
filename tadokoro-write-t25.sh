avr-gcc -Wall -Os -mmcu=attiny25 -DYJSNPI_MAKE -o tadokoro.o tadokoro.c
avr-gcc -Wall -Os -mmcu=attiny25 -DYJSNPI_MAKE -S -o tadokoro.s tadokoro.c
avr-objcopy -j .text -j .data -O ihex tadokoro.o tadokoro.hex
sudo avrdude -p t25 -C ~/YJSNPI/avrdude_gpio.conf -c pi_1 -v -U flash:w:tadokoro.hex:i
