#AUTHOR: Ted Meyer (2015)


SOURCES = devices.c blink.c




# PROJECT SPECIFIC CONFIGURATION
TARGET  = blink#  put program name here
MCU     = msp430f5529# put the model of the board here
TILIB   = CTS_Layer.c CTS_HAL.c structure.c $(SOURCES)


# SYSTEM SPECIFIC CONFIGURATION
HEADERS = /usr/local/lib/msp430# location of header files
CC      = msp430-gcc# compiler

# FLAGS
INCLUDES = -I $(HEADERS)
LINKER   = -T $(HEADERS)/$(MCU)/memory.ld -T $(HEADERS)/$(MCU)/peripherals.ld
MODEL    = -D__$(MCU)__ -mmcu=$(MCU)
WL       = -Wl,--gc-sections -Wl,-Map=$(TARGET).map
OBJECTS  = $(TILIB:.c=.o)
CFLAGS   = $(INCLUDES) $(MODEL) $(LINKER) $(WL)

all: build

build: $(OBJECTS)

.c.o:
	@echo CC -c $<
	@$(CC) -c $+ $(CFLAGS)

build:
	@echo CC -o $@
	@echo $(CFLAGS)
	@$(CC) -o $(TARGET) $+ $(CFLAGS)

clean:
	@echo cleaning
	rm -f *.o *.map $(TARGET)
