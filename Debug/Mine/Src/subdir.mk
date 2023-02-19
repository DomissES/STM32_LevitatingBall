################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Mine/Src/circ_buff.c \
../Mine/Src/font_ms_sans.c \
../Mine/Src/gui.c \
../Mine/Src/ina219.c \
../Mine/Src/lcd_service.c \
../Mine/Src/pid.c \
../Mine/Src/sh1106_hw.c \
../Mine/Src/state_machine.c \
../Mine/Src/work.c 

OBJS += \
./Mine/Src/circ_buff.o \
./Mine/Src/font_ms_sans.o \
./Mine/Src/gui.o \
./Mine/Src/ina219.o \
./Mine/Src/lcd_service.o \
./Mine/Src/pid.o \
./Mine/Src/sh1106_hw.o \
./Mine/Src/state_machine.o \
./Mine/Src/work.o 

C_DEPS += \
./Mine/Src/circ_buff.d \
./Mine/Src/font_ms_sans.d \
./Mine/Src/gui.d \
./Mine/Src/ina219.d \
./Mine/Src/lcd_service.d \
./Mine/Src/pid.d \
./Mine/Src/sh1106_hw.d \
./Mine/Src/state_machine.d \
./Mine/Src/work.d 


# Each subdirectory must supply rules for building sources it contributes
Mine/Src/%.o Mine/Src/%.su: ../Mine/Src/%.c Mine/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"D:/Projekty/Nauka_STM/PingPong/Mine" -I"D:/Projekty/Nauka_STM/PingPong/Mine/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Mine-2f-Src

clean-Mine-2f-Src:
	-$(RM) ./Mine/Src/circ_buff.d ./Mine/Src/circ_buff.o ./Mine/Src/circ_buff.su ./Mine/Src/font_ms_sans.d ./Mine/Src/font_ms_sans.o ./Mine/Src/font_ms_sans.su ./Mine/Src/gui.d ./Mine/Src/gui.o ./Mine/Src/gui.su ./Mine/Src/ina219.d ./Mine/Src/ina219.o ./Mine/Src/ina219.su ./Mine/Src/lcd_service.d ./Mine/Src/lcd_service.o ./Mine/Src/lcd_service.su ./Mine/Src/pid.d ./Mine/Src/pid.o ./Mine/Src/pid.su ./Mine/Src/sh1106_hw.d ./Mine/Src/sh1106_hw.o ./Mine/Src/sh1106_hw.su ./Mine/Src/state_machine.d ./Mine/Src/state_machine.o ./Mine/Src/state_machine.su ./Mine/Src/work.d ./Mine/Src/work.o ./Mine/Src/work.su

.PHONY: clean-Mine-2f-Src

