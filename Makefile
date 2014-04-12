# put your *.o targets here, make should handle the rest!

SRCS = main.c stm32f4xx_it.c system_stm32f4xx.c 
SRCS += controle_motores.c L3G4200D.c ADXL345.c aquisicao_IMU.c nRF24l01.c
SRCS += funcoes_spi.c array_functions.c tratamento_sinal.c
SRCS += kalman_filter.c processo_controle.c HMC5883L.c processamento_entrada.c

# all the files will be generated with this name (main.elf, main.bin, main.hex, etc)

PROJ_NAME=main

#Diretório do ST-Link

STLINK_WIN = "C:\Program Files (x86)\STMicroelectronics\STM32 ST-LINK Utility\ST-LINK Utility"

# that's it, no need to change anything below this line!

###################################################

CC=arm-none-eabi-gcc
OBJCOPY=arm-none-eabi-objcopy

CFLAGS = -DHSE_VALUE=8000000
CFLAGS += -DARM_MATH_CM4=1 -DARM_MATH_MATRIX_CHECK=1 -D__FPU_USED=1 -DUSE_STDPERIPH_DRIVER=1

CFLAGS += -g -Wall -Tstm32_flash.ld 
CFLAGS += -mlittle-endian -mthumb -mcpu=cortex-m4 -mthumb-interwork
CFLAGS += -mfloat-abi=hard -mfpu=fpv4-sp-d16
CFLAGS += -ffunction-sections -fdata-sections -O0  #Retirada do compilador TrueStudio
CFLAGS += -Wl,-Map,a.map #mapa de memória


###################################################

vpath %.c src
vpath %.a lib

ROOT=$(shell pwd)

CFLAGS += -Iinc -Ilib -Ilib/inc 
CFLAGS += -Ilib/inc/core -Ilib/inc/peripherals 

SRCS += lib/startup_stm32f4xx.s # add startup file to build

OBJS = $(SRCS:.c=.o)

###################################################

.PHONY: lib proj

all: lib proj

lib:
	$(MAKE) -C lib

proj: 	$(PROJ_NAME).elf

$(PROJ_NAME).elf: $(SRCS)
	$(CC) $(CFLAGS) $^ -o $@ -Llib -lstm32f4 -larm_cortexM4lf_math -lm
	$(OBJCOPY) -O ihex $(PROJ_NAME).elf $(PROJ_NAME).hex
	$(OBJCOPY) -O binary $(PROJ_NAME).elf $(PROJ_NAME).bin

clean:
	rm -f *.o
	rm -f $(PROJ_NAME).elf
	rm -f $(PROJ_NAME).hex
	rm -f $(PROJ_NAME).bin

again: clean all

resetl:

burnl:
	st-flash write main.bin 0x08000000

resetw:
	$(STLINK_WIN)/ST-LINK_CLI.exe -Rst -Run

burnw:
	$(STLINK_WIN)/ST-LINK_CLI.exe -P main.hex 0x08000000 -Rst -Run
	