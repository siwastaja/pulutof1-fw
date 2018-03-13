# This makefile is made to work with the toolchain downloadable at https://launchpad.net/gcc-arm-embedded

CC = arm-none-eabi-gcc
LD = arm-none-eabi-gcc
SIZE = arm-none-eabi-size
OBJCOPY = arm-none-eabi-objcopy

CFLAGS = -I. -Os -fno-common -ffunction-sections -ffreestanding -fno-builtin -mthumb -mcpu=cortex-m7 -specs=nano.specs -Wall -fstack-usage -DSTM32F765xx -mfloat-abi=hard -mfpu=fpv5-d16 -fno-strict-aliasing -Wno-discarded-qualifiers
ASMFLAGS = -S -fverbose-asm
LDFLAGS = -mcpu=cortex-m7 -mthumb -nostartfiles -gc-sections -mfloat-abi=hard -mfpu=fpv5-d16 -specs=nano.specs

DEPS =
OBJ = stm32init.o main.o own_std.o tof_table.o flash.o
ASMS = stm32init.s main.s own_std.s tof_table.s flash.s

all: main.bin

%.o: %.c $(DEPS)
	$(CC) -c -o $@ $< $(CFLAGS)

main.bin: $(OBJ)
	$(LD) -Tstm32.ld $(LDFLAGS) -o main.elf $^ -lm
	$(OBJCOPY) -Obinary --remove-section=.ARM* --remove-section=*bss* --remove-section=.settings main.elf main_full.bin
	$(SIZE) main.elf

# NOTE: I need a custom bss section in the core-coupled DTCM RAM. This should be a piece of cake, but it isn't.
# Tried to find a way to tell GCC/LD to mark this bss custom bss section with ALLOC flag only, but it always gets generated with CONTENTS, ALLOC, LOAD, DATA,
# causing the binary file to extend to the address space of the totally wrong (meaningless) "load address", generating 500 megabyte binary.
# Since bss sections have nothing to do in the binary, we can just as well remove them from the output file.
# .settings is also removed - this makes the binary small, and keeps the old settings

fr: main.bin
#	scp main_full.bin hrst@192.168.130.149:~/main_full.bin
	scp main_full.bin hrst@192.168.1.3:~/main_full.bin

flash_full: main.bin
	stm32sprog -b 115200 -vw main_full.bin

flash: main.bin
	stm32sprog -b 115200 -vw main_full.bin

stack:
	cat *.su

sections:
	arm-none-eabi-objdump -h main.elf

syms:
	arm-none-eabi-objdump -t main.elf

%.s: %.c $(DEPS)
	$(CC) -c -o $@ $< $(CFLAGS) $(ASMFLAGS)

asm: $(ASMS)

e: 
	gedit --new-window stm32.ld stm32init.c main.c flash.c flash.h &

s:
	screen /dev/ttyUSB0 115200
