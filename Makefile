# ELF target name
TARGET=quad.elf

# C source files
SRC=src/system_stm32f4xx.c

# C++ source files
CXXSRC=src/main.cpp
CXXSRC+=src/BaseTimeControl.cpp
CXXSRC+=hal/src/stm32f4gpiohal.cpp
CXXSRC+=hal/src/stm32f4leds.cpp
CXXSRC+=hal/src/stm32f4spihal.cpp
CXXSRC+=hal/src/stm32f4i2chal.cpp
CXXSRC+=hal/src/NRF24L01P.cpp
CXXSRC+=src/stm32f4xx_it.cpp

# ASM source files
ASRC=lib/startup_stm32f4xx.s

# include directories
INCDIRS=inc lib lib/inc lib/inc/core lib/inc/peripherals hal/include

# search path for .so and .a
LIBDIRS=lib

# libraries to link
LIBS=stm32f4 c g m arm_cortexM4lf_math

# Linker script
LDSCRIPT=stm32_flash.ld

# Tool configuration
CROSS=arm-none-eabi-
CC=$(CROSS)gcc
CXX=$(CROSS)g++
AS=$(CROSS)gcc
LD=$(CROSS)gcc
OBJCOPY=$(CROSS)objcopy

# Architecture configuration
ARCH_FLAGS=  -mlittle-endian
ARCH_FLAGS+= -mcpu=cortex-m4
ARCH_FLAGS+= -mthumb
ARCH_FLAGS+= -mfloat-abi=hard
ARCH_FLAGS+= -mfpu=fpv4-sp-d16

# Configurações para as bibliotecas (Defines globais)
LIB_DEFINES+= -DHSE_VALUE=8000000
LIB_DEFINES+= -DARM_MATH_CM4=1
LIB_DEFINES+= -DARM_MATH_MATRIX_CHECK=1
LIB_DEFINES+= -D__FPU_USED=1
LIB_DEFINES+= -DUSE_STDPERIPH_DRIVER=1

# Flags for gcc
CFLAGS+= -ggdb		#Ativa as opções para debug
CFLAGS+= -O0
CFLAGS+=$(ARCH_FLAGS)
CFLAGS+=$(LIB_DEFINES)
#CFLAGS+=-flto
CFLAGS+= -ffunction-sections
CFLAGS+= -fdata-sections
CFLAGS+= -Wall
CFLAGS+=$(foreach i, $(INCDIRS), -I$(i))

# Flags for g++
CXXFLAGS=$(CFLAGS)
CXXFLAGS+=-fno-rtti
CXXFLAGS+=-fno-exceptions
CXXFLAGS+=-std=c++11


# Flags for gcc as linker
LDFLAGS=$(ARCH_FLAGS)
LDFLAGS+=-Wl,--gc-sections
LDFLAGS+=$(foreach i, $(LIBDIRS), -L$(i))
LDFLAGS+=-specs=nosys.specs
LDFLAGS+=-T $(LDSCRIPT)

# Librarys to Link
LDLIB=$(foreach i, $(LIBS), -l$(i))

# No man land! Enter at your risk

OBJS=$(SRC:.c=.o) $(ASRC:.s=.o) $(CXXSRC:.cpp=.o)
DEPS=$(OBJS:.o=.d)

# Default make target (first target)
all: $(TARGET) bins

# include dependency files (*.d)
-include $(DEPS)

# Rules to build c/cpp/s files
%.o: %.c
	@echo " CC $<"
	@$(CC) -MMD $(CFLAGS) -o $@ -c $<

%.o: %.cpp
	@echo " CXX $<"
	@$(CXX) -MMD $(CXXFLAGS) -o $@ -c $<

%.o: %.s
	@echo " AS $<"
	@$(AS) $(CFLAGS) -o $@ -c $<

# Linker rule
$(TARGET): libs $(OBJS)
	@echo " LINK $@"
	@$(CXX) -o $@ $(OBJS) $(LDFLAGS) $(LDLIB)

# Phony rules (not file-asociated)
.PHONY: clean all libs clean_lib bins

libs:
	@$(MAKE) -C lib

clean_lib:
	@$(MAKE) -C lib clean

clean:
	@echo " CLEAN"
	@rm -fR $(OBJS) $(DEPS) $(TARGET) $(TARGET:.elf=hex) $(TARGET:.elf=.bin)

bins: $(TARGET)
	@echo " BUILD HEX & BIN"
	@$(OBJCOPY) -O ihex $(TARGET) $(TARGET:.elf=.hex)
	@$(OBJCOPY) -O binary $(TARGET) $(TARGET:.elf=.bin)
