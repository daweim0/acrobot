BASE = `ls old_controller/beta*|wc -l`
NUM = `echo $(BASE)+1|bc`
PROJECT=balance
SOURCES=uart.c balance.c

CC=avr-gcc
STRIP=avr-strip
OBJCOPY=avr-objcopy
MMCU=atmega128
USBDEV=$(firstword $(wildcard /dev/ttyUSB*))

CFLAGS=-mmcu=$(MMCU) -Wall -DF_CPU=16000000UL -I.. -funsigned-char -funsigned-bitfields -fpack-struct -fshort-enums -g -gstabs+ -Os -finline-functions
DEFS=-DSTREAM

$(PROJECT).hex: $(PROJECT).out
	$(OBJCOPY) -j .text -j .data -O ihex $(PROJECT).out $(PROJECT).hex

$(PROJECT).out: $(SOURCES)
	$(CC) $(CFLAGS) $(DEFS) -I. -o $(PROJECT).out $(SOURCES)

%.out: %.c
	$(CC) $(CFLAGS) $(DEFS) -I. -o $@ $<

%.s: %.c
	$(CC) $(CFLAGS) $(DEFS) -I. -S $<

asm: $(SOURCES)
	$(CC) $(CFLAGS) -S $(DEFS) -I. $(SOURCES)

install: $(PROJECT).hex
	#avrdude -p m128 -c avrisp -P /dev/ttyUSB1 -b 19200 -e -U flash:w:$(PROJECT).hex -B 16
	avrdude -p m128 -c jtag2 -P usb -e -U flash:w:$(PROJECT).hex -B 16
	@echo -n "i" >> write_count.txt
	@echo "Writes: " `wc -c write_count.txt`

usbinstall: $(PROJECT).hex
	avrdude -p m128 -c avrisp -P $(USBDEV) -b 19200 -U flash:w:$(PROJECT).hex
	@echo -n "i" >> write_count.txt
	@echo "Writes: " `wc -c write_count.txt`
clean:
	rm -f *.hex *.out

update:
	echo $(NUM)
	#mv beta.txt old_controller/beta$(NUM).txt
	#scp jyoungqu@slfiles02:/shareddata/jay/balance_bot/*.txt ./

.PHONY: clean

