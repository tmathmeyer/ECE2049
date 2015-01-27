CC       = msp430-gcc
MCU      = msp430f5529
TARGET   = blink
INC_DIR  = /home/ted/git/ECE2049/testing/include
INCLUDES = -I $(INC_DIR)

SOURCES  = blink.c
OBJECTS  = $(SOURCES:.c=.o)

CFLAGS   = $(INCLUDES) -D__$(MCU)__ -mmcu=$(MCU)  -T $(INC_DIR)/$(MCU)/memory.ld -T $(INC_DIR)/$(MCU)/peripherals.ld -Wl,--gc-sections -Wl,-Map=$(TARGET).map


all: blink.o
	$(CC) $(OBJECTS) -o $(TARGET).elf $(CFLAGS)

clean:
	rm *.o *.map
