################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/freertos.c \
../Src/main.c \
../Src/stm32f1xx_hal_msp.c \
../Src/stm32f1xx_hal_timebase_TIM.c \
../Src/stm32f1xx_it.c \
../Src/system_stm32f1xx.c \
../Src/usb_device.c \
../Src/usbd_cdc_if.c \
../Src/usbd_conf.c \
../Src/usbd_desc.c 

OBJS += \
./Src/freertos.o \
./Src/main.o \
./Src/stm32f1xx_hal_msp.o \
./Src/stm32f1xx_hal_timebase_TIM.o \
./Src/stm32f1xx_it.o \
./Src/system_stm32f1xx.o \
./Src/usb_device.o \
./Src/usbd_cdc_if.o \
./Src/usbd_conf.o \
./Src/usbd_desc.o 

C_DEPS += \
./Src/freertos.d \
./Src/main.d \
./Src/stm32f1xx_hal_msp.d \
./Src/stm32f1xx_hal_timebase_TIM.d \
./Src/stm32f1xx_it.d \
./Src/system_stm32f1xx.d \
./Src/usb_device.d \
./Src/usbd_cdc_if.d \
./Src/usbd_conf.d \
./Src/usbd_desc.d 


# Each subdirectory must supply rules for building sources it contributes
Src/%.o: ../Src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m3 -mthumb -mfloat-abi=soft '-D__weak=__attribute__((weak))' '-D__packed="__attribute__((__packed__))"' -DUSE_HAL_DRIVER -DSTM32F103xB -I"/home/gastd/sandbox/stm32/test_com1/Inc" -I"/home/gastd/sandbox/stm32/test_com1/Drivers/STM32F1xx_HAL_Driver/Inc" -I"/home/gastd/sandbox/stm32/test_com1/Drivers/STM32F1xx_HAL_Driver/Inc/Legacy" -I"/home/gastd/sandbox/stm32/test_com1/Drivers/CMSIS/Device/ST/STM32F1xx/Include" -I"/home/gastd/sandbox/stm32/test_com1/Drivers/CMSIS/Include" -I"/home/gastd/sandbox/stm32/test_com1/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM3" -I"/home/gastd/sandbox/stm32/test_com1/Middlewares/ST/STM32_USB_Device_Library/Core/Inc" -I"/home/gastd/sandbox/stm32/test_com1/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc" -I"/home/gastd/sandbox/stm32/test_com1/Middlewares/Third_Party/FreeRTOS/Source/include" -I"/home/gastd/sandbox/stm32/test_com1/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS"  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


