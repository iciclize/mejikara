avr-gcc -Wall -Os -mmcu=attiny85 -o pwm.o pwm.c
avr-objcopy -j .text -j .data -O ihex pwm.o pwm.hex
sudo avrdude -p t85 -C ~/YJSNPI/avrdude_gpio.conf -c pi_1 -v -U flash:w:pwm.hex:i
