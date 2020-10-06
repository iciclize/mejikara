avr-gcc -Wall -Os -mmcu=attiny85 -o yjsnpi.o yjsnpi.c
avr-objcopy -j .text -j .data -O ihex yjsnpi.o yjsnpi.hex
sudo avrdude -p t85 -C ~/YJSNPI/avrdude_gpio.conf -c pi_1 -v -U flash:w:yjsnpi.hex:i
