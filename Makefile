build: tadokoro.o
	make tadokoro.o
	make tadokoro.s

tadokoro.o: tadokoro.c
	avr-gcc -Wall -Os -mmcu=attiny25 -DYJSNPI_MAKE -o tadokoro.o tadokoro.c

tadokoro.s: tadokoro.c
	avr-gcc -Wall -Os -mmcu=attiny25 -DYJSNPI_MAKE -S -o tadokoro.s tadokoro.c

tadokoro.hex: tadokoro.o
	avr-objcopy -j .text -j .data -O ihex tadokoro.o tadokoro.hex

write: tadokoro.hex
	sudo avrdude -p t25 -C ./avrdude_gpio.conf -c pi_1 -v -U flash:w:tadokoro.hex:i
