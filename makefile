
MAIN = PIDControl

FLASH.bin : $(MAIN).elf
	avr-objcopy -O binary $(MAIN).elf FLASH.bin

$(MAIN).elf : $(MAIN).o Motors.o Sensors.o
	avr-gcc -mmcu=atmega32u4 $(MAIN).o Motors.o Sensors.o -o $(MAIN).elf

$(MAIN).o : $(MAIN).c Motors.h Sensors.h makefile
	avr-gcc -O1 -mmcu=atmega32u4 -c $(MAIN).c -o $(MAIN).o

Motors.o : Motors.c
	avr-gcc -O1 -mmcu=atmega32u4 -c Motors.c -o Motors.o

Sensors.o : Sensors.c
	avr-gcc -O1 -mmcu=atmega32u4 -c Sensors.c -o Sensors.o
