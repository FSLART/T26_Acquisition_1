################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/adc.c \
../Core/Src/analog.c \
../Core/Src/app.c \
../Core/Src/can.c \
../Core/Src/can_bus.c \
../Core/Src/can_messages.c \
../Core/Src/debug.c \
../Core/Src/digital_inputs.c \
../Core/Src/dma.c \
../Core/Src/gpio.c \
../Core/Src/iwdg.c \
../Core/Src/main.c \
../Core/Src/stm32f4xx_hal_msp.c \
../Core/Src/stm32f4xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32f4xx.c \
../Core/Src/tim.c \
../Core/Src/timebase.c \
../Core/Src/usart.c \
../Core/Src/watchdog.c 

OBJS += \
./Core/Src/adc.o \
./Core/Src/analog.o \
./Core/Src/app.o \
./Core/Src/can.o \
./Core/Src/can_bus.o \
./Core/Src/can_messages.o \
./Core/Src/debug.o \
./Core/Src/digital_inputs.o \
./Core/Src/dma.o \
./Core/Src/gpio.o \
./Core/Src/iwdg.o \
./Core/Src/main.o \
./Core/Src/stm32f4xx_hal_msp.o \
./Core/Src/stm32f4xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32f4xx.o \
./Core/Src/tim.o \
./Core/Src/timebase.o \
./Core/Src/usart.o \
./Core/Src/watchdog.o 

C_DEPS += \
./Core/Src/adc.d \
./Core/Src/analog.d \
./Core/Src/app.d \
./Core/Src/can.d \
./Core/Src/can_bus.d \
./Core/Src/can_messages.d \
./Core/Src/debug.d \
./Core/Src/digital_inputs.d \
./Core/Src/dma.d \
./Core/Src/gpio.d \
./Core/Src/iwdg.d \
./Core/Src/main.d \
./Core/Src/stm32f4xx_hal_msp.d \
./Core/Src/stm32f4xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32f4xx.d \
./Core/Src/tim.d \
./Core/Src/timebase.d \
./Core/Src/usart.d \
./Core/Src/watchdog.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F412Rx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/adc.cyclo ./Core/Src/adc.d ./Core/Src/adc.o ./Core/Src/adc.su ./Core/Src/analog.cyclo ./Core/Src/analog.d ./Core/Src/analog.o ./Core/Src/analog.su ./Core/Src/app.cyclo ./Core/Src/app.d ./Core/Src/app.o ./Core/Src/app.su ./Core/Src/can.cyclo ./Core/Src/can.d ./Core/Src/can.o ./Core/Src/can.su ./Core/Src/can_bus.cyclo ./Core/Src/can_bus.d ./Core/Src/can_bus.o ./Core/Src/can_bus.su ./Core/Src/can_messages.cyclo ./Core/Src/can_messages.d ./Core/Src/can_messages.o ./Core/Src/can_messages.su ./Core/Src/debug.cyclo ./Core/Src/debug.d ./Core/Src/debug.o ./Core/Src/debug.su ./Core/Src/digital_inputs.cyclo ./Core/Src/digital_inputs.d ./Core/Src/digital_inputs.o ./Core/Src/digital_inputs.su ./Core/Src/dma.cyclo ./Core/Src/dma.d ./Core/Src/dma.o ./Core/Src/dma.su ./Core/Src/gpio.cyclo ./Core/Src/gpio.d ./Core/Src/gpio.o ./Core/Src/gpio.su ./Core/Src/iwdg.cyclo ./Core/Src/iwdg.d ./Core/Src/iwdg.o ./Core/Src/iwdg.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/stm32f4xx_hal_msp.cyclo ./Core/Src/stm32f4xx_hal_msp.d ./Core/Src/stm32f4xx_hal_msp.o ./Core/Src/stm32f4xx_hal_msp.su ./Core/Src/stm32f4xx_it.cyclo ./Core/Src/stm32f4xx_it.d ./Core/Src/stm32f4xx_it.o ./Core/Src/stm32f4xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32f4xx.cyclo ./Core/Src/system_stm32f4xx.d ./Core/Src/system_stm32f4xx.o ./Core/Src/system_stm32f4xx.su ./Core/Src/tim.cyclo ./Core/Src/tim.d ./Core/Src/tim.o ./Core/Src/tim.su ./Core/Src/timebase.cyclo ./Core/Src/timebase.d ./Core/Src/timebase.o ./Core/Src/timebase.su ./Core/Src/usart.cyclo ./Core/Src/usart.d ./Core/Src/usart.o ./Core/Src/usart.su ./Core/Src/watchdog.cyclo ./Core/Src/watchdog.d ./Core/Src/watchdog.o ./Core/Src/watchdog.su

.PHONY: clean-Core-2f-Src

