avr-gcc -mmcu=attiny2313a -Wall -Os -o ./bin/test.elf ./src/test.c
avr-objcopy -O ihex -R .eeprom ./bin/test.elf ./bin/test.hex
avrdude -c usbasp -p t2313 -U flash:w:./bin/test.hex
