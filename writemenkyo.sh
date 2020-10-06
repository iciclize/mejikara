avr-gcc -Wall -Os -mmcu=attiny85 -o menkyo.o menkyo.c
avr-objcopy -j .text -j .data -O ihex menkyo.o menkyo.hex
sudo avrdude -p t85 -C ~/YJSNPI/avrdude_gpio.conf -c pi_1 -v -U flash:w:menkyo.hex:i
