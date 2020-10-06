avr-gcc -Wall -Os -mmcu=attiny25 -DYJSNPI_MAKE -o dededon.o dededon.c
avr-objcopy -j .text -j .data -O ihex dededon.o dededon.hex
sudo avrdude -p t25 -C ~/YJSNPI/avrdude_gpio.conf -c pi_1 -v -U flash:w:dededon.hex:i
