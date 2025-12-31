################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Inc/ADCconverter/ring_buffer/ring_buffer.c 

OBJS += \
./Core/Inc/ADCconverter/ring_buffer/ring_buffer.o 

C_DEPS += \
./Core/Inc/ADCconverter/ring_buffer/ring_buffer.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Inc/ADCconverter/ring_buffer/%.o Core/Inc/ADCconverter/ring_buffer/%.su Core/Inc/ADCconverter/ring_buffer/%.cyclo: ../Core/Inc/ADCconverter/ring_buffer/%.c Core/Inc/ADCconverter/ring_buffer/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F401xC -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Inc-2f-ADCconverter-2f-ring_buffer

clean-Core-2f-Inc-2f-ADCconverter-2f-ring_buffer:
	-$(RM) ./Core/Inc/ADCconverter/ring_buffer/ring_buffer.cyclo ./Core/Inc/ADCconverter/ring_buffer/ring_buffer.d ./Core/Inc/ADCconverter/ring_buffer/ring_buffer.o ./Core/Inc/ADCconverter/ring_buffer/ring_buffer.su

.PHONY: clean-Core-2f-Inc-2f-ADCconverter-2f-ring_buffer

