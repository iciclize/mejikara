avr-gcc -Wall -Os -mmcu=attiny85 -o dededon.o dededon.c
avr-objcopy -j .text -j .data -O ihex dededon.o dededon.hex
sudo avrdude -p t85 -C ~/YJSNPI/avrdude_gpio.conf -c pi_1 -v -U flash:w:dededon.hex:i
