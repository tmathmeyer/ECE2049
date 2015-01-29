#AUTHOR: Ted Meyer (2015)
SOURCES = blink.c devices.c


# PROJECT SPECIFIC CONFIGURATION
TARGET  = blink#  put program name here
MCU     = msp430f5529# put the model of the board here
TILIB   = CTS_Layer.c structure.c CTS_HAL.c
SRCALL  = $(SOURCES) $(TILIB)


# SYSTEM SPECIFIC CONFIGURATION
HEADERS = /usr/local/lib/msp430# location of header files
CC      = msp430-gcc# compiler

# FLAGS
INCLUDES = -I $(HEADERS)
#MEMORY   = -T $(HEADERS)/$(MCU)/memory.ld
MEMORY   = -T /home/ted/git/ECE2049/labs/space_invaders_tjmeyer/lnk_msp430f5529.cmd

PERPH    = -T $(HEADERS)/$(MCU)/peripherals.ld
LINKER   = $(MEMORY) $(PERPH)
MODEL    = -D__$(MCU)__ -mmcu=$(MCU)
WL       = -Wl,--gc-sections -Wl,-Map=$(TARGET).map
OBJECTS  = $(SRCALL:.c=.o)
CFLAGS   = $(INCLUDES) $(MODEL)

all: build

build: $(OBJECTS)

.c.o:
	@echo CC -c $<
	@$(CC) -c $+ $(CFLAGS)

build:
	@echo CC -o $(TARGET) $+
	@$(CC) -o $(TARGET) $+ $(CFLAGS) $(WL) $(LINKER)

clean:
	@echo cleaning
	rm -f *.o *.map $(TARGET)
