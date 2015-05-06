# Makefile configuration file





# Name of Output file
MAIN_OUT = Kamami_Stm32_blinky




# System stack size 
SYSTEM_STACK_SIZE = 256



# Output file format
FORMAT = ihex




# Optimisation level (option: 0, 1, 2, 3)
COMPILE_OPTS = -mcpu=cortex-m3 -mthumb -Wall -g -O0





# Folder contains user code
INCLUDE_DIRS = -I . -I Include
INCLUDE_DIRS += -I . -I Src



# FreeRTOS include library
INCLUDE_DIRS += -I . -I FreeRTOS_Header
INCLUDE_DIRS += -I . -I FreeRTOS_Header/include
INCLUDE_DIRS += -I . -I FreeRTOS_Header/Common/include
INCLUDE_DIRS += -I . -I FreeRTOS_Source
INCLUDE_DIRS += -I . -I FreeRTOS_Source/portable/GCC/ARM_CM3
INCLUDE_DIRS += -I . -I FreeRTOS_Source/portable/MemMang




# Folder contains ST Microelectronic libraries
INCLUDE_DIRS += -I . -I STLibrary/inc
INCLUDE_DIRS += -I . -I STLibrary/src





# Folder contain linker's scripts
LIBRARY_DIRS += -L Ldscripts
LIBRARY_DIRS += -L STLibrary






# Folder which contain output files *.hex, *.elf, *.bin *.lss etc. 
OUT_DIR = ./Output/




# Linker's script
LNKSCRIPT = ./Ldscripts/




# User files declaration
LIBRARY = Src/main.o
LIBRARY += Src/stm32f10x_it.o
LIBRARY += Src/Stm32l_level.o
LIBRARY += Src/Lcd3310.o
LIBRARY += Src/StringsConv.o

LIBRARY += Src/ParTest.o
LIBRARY += Src/timertest.o
LIBRARY += FreeRTOS_Source/croutine.o
LIBRARY += FreeRTOS_Source/event_groups.o
LIBRARY += FreeRTOS_Source/list.o
LIBRARY += FreeRTOS_Source/queue.o
LIBRARY += FreeRTOS_Source/tasks.o
LIBRARY += FreeRTOS_Source/timers.o
LIBRARY += FreeRTOS_Source/portable/GCC/ARM_CM3/port.o
LIBRARY += FreeRTOS_Source/Full/BlockQ.o
LIBRARY += FreeRTOS_Source/Full/comtest.o
LIBRARY += FreeRTOS_Source/Full/death.o
LIBRARY += FreeRTOS_Source/Full/dynamic.o
LIBRARY += FreeRTOS_Source/Full/events.o
LIBRARY += FreeRTOS_Source/Full/flash.o
LIBRARY += FreeRTOS_Source/Full/integer.o
LIBRARY += FreeRTOS_Source/Full/PollQ.o
LIBRARY += FreeRTOS_Source/Full/print.o
LIBRARY += FreeRTOS_Source/Full/semtest.o
LIBRARY += FreeRTOS_Source/Full/QPeek.o
LIBRARY += FreeRTOS_Source/Full/GenQTest.o
LIBRARY += FreeRTOS_Source/Full/recmutex.o
#LIBRARY += FreeRTOS_Source/Full/crhook.o
LIBRARY += FreeRTOS_Source/portable/MemMang/heap_2.o
LIBRARY += Src/syscalls.o





# Libraries supplied by manufacturer ST Microelectronics
#LIBRARY += STLibrary/src/stm32f10x_adc.o
#LIBRARY += STLibrary/src/stm32f10x_bkp.o
#LIBRARY += STLibrary/src/stm32f10x_can.o
#LIBRARY += STLibrary/src/stm32f10x_dma.o
#LIBRARY += STLibrary/src/stm32f10x_exti.o
#LIBRARY += STLibrary/src/stm32f10x_flash.o
LIBRARY += STLibrary/src/stm32f10x_gpio.o
#LIBRARY += STLibrary/src/stm32f10x_i2c.o
#LIBRARY += STLibrary/src/stm32f10x_iwdg.o
#LIBRARY += STLibrary/src/stm32f10x_lib.o
LIBRARY += STLibrary/src/stm32f10x_nvic.o
#LIBRARY += STLibrary/src/stm32f10x_pwr.o
LIBRARY += STLibrary/src/stm32f10x_rcc.o
#LIBRARY += STLibrary/src/stm32f10x_rtc.o
#LIBRARY += STLibrary/src/stm32f10x_spi.o
LIBRARY += STLibrary/src/stm32f10x_systick.o
LIBRARY += STLibrary/src/stm32f10x_tim.o
#LIBRARY += STLibrary/src/stm32f10x_tim1.o
#LIBRARY += STLibrary/src/stm32f10x_usart.o
#LIBRARY += STLibrary/src/stm32f10x_wwdg.o
LIBRARY += STLibrary/src/cortexm3_macro.o
LIBRARY += STLibrary/src/stm32f10x_vector.o

# Add math library
#MATH_LIB = -lm
#CPLUSPLUS_LIB = -lstdc++

CC = arm-none-eabi-gcc
CFLAGS = $(COMPILE_OPTS) $(INCLUDE_DIRS)

CXX = arm-none-eabi-g++
CXXFLAGS = $(COMPILE_OPTS) $(INCLUDE_DIRS)
CXXFLAGS +=-fno-exceptions
CXXFLAGS +=-fno-rtti

# Allow to debug in C++
CXXFLAGS += -g

AS = arm-none-eabi-gcc
ASFLAGS = $(COMPILE_OPTS) -c

LD = arm-none-eabi-gcc
#LD = arm-none-eabi-g++
# Change following option to obtain proper (-Map=$@.map) map file in Output directory 
#LDFLAGS = -Wl,--gc-sections,-Map=$@.map,-cref,-u,Reset_Handler $(INCLUDE_DIRS) $(LIBRARY_DIRS) $(CPLUSPLUS_LIB) -T $(LNKSCRIPT)stm32.ld
LDFLAGS = -Wl,--defsym=_sys_stack_size=$(SYSTEM_STACK_SIZE),--gc-sections,-Map=$(OUT_DIR)$(MAIN_OUT).map,-cref,-u,Reset_Handler $(INCLUDE_DIRS) $(LIBRARY_DIRS) $(CPLUSPLUS_LIB) -T $(LNKSCRIPT)stm32.ld

# Add this option if C++ want to be used for Cortex only in Thumb mode
LDFLAGS += -mthumb

OBJCP = arm-none-eabi-objcopy
#OBJCPFLAGS = -O binary

OBJCPFLAGSH = -O ihex

AR = arm-none-eabi-ar
ARFLAGS = cr

# File siez after compilation
SIZE = arm-none-eabi-size


MAIN_OUT_ELF = $(OUT_DIR)$(MAIN_OUT).elf
MAIN_OUT_BIN = $(OUT_DIR)$(MAIN_OUT).bin
MAIN_OUT_HEX = $(OUT_DIR)$(MAIN_OUT).hex

MSG_SIZE_BEFORE = Size before compilation process: 
MSG_SIZE_AFTER = Size after compilation process:
ODZIELENIE_DIALOGOW = --------------------------------------------------------------

# all

#all: gccversion $(MAIN_OUT_ELF) $(MAIN_OUT_BIN) sizeafter
all: gccversion $(MAIN_OUT_ELF) $(MAIN_OUT_HEX) sizeafter  

# main

# Show information about compiler version
gccversion : 
	@$(CC) --version; echo $(ODZIELENIE_DIALOGOW);

#$(MAIN_OUT_ELF): $(MAIN_OUT).o $(LIBRARY) 
#$(LD) $(LDFLAGS) $(MAIN_OUT).o $(LIBRARY) --output $@  
#@echo $(ODZIELENIE_DIALOGOW);

$(MAIN_OUT_ELF): $(LIBRARY) 
	$(LD) $(LDFLAGS) $(LIBRARY) --output $@  
	@echo $(ODZIELENIE_DIALOGOW);

$(MAIN_OUT_BIN): $(MAIN_OUT_ELF)
	$(OBJCP) $(OBJCPFLAGS) $< $@

$(MAIN_OUT_HEX): $(MAIN_OUT_ELF)
	$(OBJCP) $(OBJCPFLAGSH) $< $@


# Display file size.
#BINNARYSIZE = $(SIZE) --target=$(FORMAT) $(OUT_DIR)$(MAIN_OUT).bin
ELFSIZE = $(SIZE) -A $(OUT_DIR)$(MAIN_OUT).elf

sizebefore:
	@if [ -f $(OUT_DIR)$(MAIN_OUT).elf ]; then echo; echo $(MSG_SIZE_BEFORE); $(ELFSIZE); echo; fi

sizeafter:
	@if [ -f $(OUT_DIR)$(MAIN_OUT).elf ]; then echo; echo $(MSG_SIZE_AFTER); $(ELFSIZE); echo; fi
	@echo $(ODZIELENIE_DIALOGOW);

# flash

flash: $(MAIN_OUT)
	@cp $(MAIN_OUT_ELF) jtag/flash
	@cd jtag; openocd -f flash.cfg
	@rm jtag/flash

clean:
#-rm *.o lib/src/*.o $(LIBSTM32_OUT) $(MAIN_OUT_ELF) $(MAIN_OUT_BIN)
#-rm $(LIBRARY)*.o $(LIBSTM32_OUT) $(MAIN_OUT_ELF) $(MAIN_OUT_BIN)
	-rm $(LIBRARY) $(MAIN_OUT_ELF) $(MAIN_OUT_HEX)
