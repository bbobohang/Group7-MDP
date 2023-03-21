################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../PeripheralDriver/Src/ICM20948.c \
../PeripheralDriver/Src/oled.c 

OBJS += \
./PeripheralDriver/Src/ICM20948.o \
./PeripheralDriver/Src/oled.o 

C_DEPS += \
./PeripheralDriver/Src/ICM20948.d \
./PeripheralDriver/Src/oled.d 


# Each subdirectory must supply rules for building sources it contributes
PeripheralDriver/Src/%.o PeripheralDriver/Src/%.su: ../PeripheralDriver/Src/%.c PeripheralDriver/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I"C:/Users/jin_c/STM32CubeIDE/workspace_1.11.0/MDP_Project/PeripheralDriver/Inc" -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-PeripheralDriver-2f-Src

clean-PeripheralDriver-2f-Src:
	-$(RM) ./PeripheralDriver/Src/ICM20948.d ./PeripheralDriver/Src/ICM20948.o ./PeripheralDriver/Src/ICM20948.su ./PeripheralDriver/Src/oled.d ./PeripheralDriver/Src/oled.o ./PeripheralDriver/Src/oled.su

.PHONY: clean-PeripheralDriver-2f-Src

