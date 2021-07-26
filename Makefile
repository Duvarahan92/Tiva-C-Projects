####################################

#Makefile, for tm4c programming 

# Target name:
TARGET = blinky

# Target type
MCU = tm4c123GH6PM
MACH = cortex-m4
MODULES = Applications/$(TARGET) libs drivers
SRC = $(foreach m, $(MODULES), $(wildcard $(m)/*.c))
OBJ = $(addprefix obj/,$(notdir $(SRC:%.c=%.o)))
BIN = bin/
INC = -I inc -I drivers

# Linker script
LINKER_SCRIPT = ld/$(MCU).ld


# Utility variables
CC = arm-none-eabi-gcc
OBJCOPY = arm-none-eabi-objcopy
MD =  @mkdir -p $(@D)    


# GCC flags
CFLAGS = -c -mcpu=$(MACH) -mthumb -mfloat-abi=soft -std=gnu11 -Wall -O0 -g
LDFLAGS = -mcpu=$(MACH) -mthumb -mfloat-abi=soft --specs=rdimon.specs -T $(LINKER_SCRIPT) -Wl,-Map=$(BIN)$(TARGET).map



#######################################

# Makerules:

compile: $(BIN)$(TARGET).bin

flash:	clean compile
	openocd -f board/ti_ek-tm4c123gxl.cfg \
	-c "program $(BIN)$(TARGET).bin reset exit 0x00000000"
erase:
	openocd -f board/ti_ek-tm4c123gxl.cfg \
	-c "init; reset halt; stellaris mass_erase 0; exit"	

debug:	clean compile
	openocd -f board/ti_ek-tm4c123gxl.cfg 
	#gdb-multiarch $(BIN)$(TARGET).bin

clean:
	rm -rf obj bin

######################################

# Psudorules:

obj/%.o: Applications/$(TARGET)/%.c
	$(MD)
	$(CC) $(INC) $(CFLAGS) -o $@ $^

obj/%.o: libs/%.c
	$(MD)
	$(CC) $(CFLAGS) -o $@ $^

obj/%.o: drivers/%.c
	$(MD)
	$(CC) $(INC) $(CFLAGS) -o $@ $^


$(BIN)$(TARGET).elf: $(OBJ)
	$(MD)
	$(CC) $(LDFLAGS) -o $@ $^

$(BIN)$(TARGET).bin: $(BIN)$(TARGET).elf
	$(OBJCOPY) -O binary $^ $@


.PHONY: compile flash debug clean
