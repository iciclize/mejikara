avr-gcc -Wall -Os -mmcu=attiny85 -o mejikara.o mejikara.c
avr-objcopy -j .text -j .data -O ihex mejikara.o mejikara.hex
sudo avrdude -p t85 -C ~/YJSNPI/avrdude_gpio.conf -c pi_1 -v -U flash:w:mejikara.hex:i
