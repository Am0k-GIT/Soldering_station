################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Inc/RingBuffer/ring_buffer.c 

OBJS += \
./Core/Inc/RingBuffer/ring_buffer.o 

C_DEPS += \
./Core/Inc/RingBuffer/ring_buffer.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Inc/RingBuffer/%.o Core/Inc/RingBuffer/%.su Core/Inc/RingBuffer/%.cyclo: ../Core/Inc/RingBuffer/%.c Core/Inc/RingBuffer/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -DUSE_HAL_DRIVER -DSTM32F401xC -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Inc-2f-RingBuffer

clean-Core-2f-Inc-2f-RingBuffer:
	-$(RM) ./Core/Inc/RingBuffer/ring_buffer.cyclo ./Core/Inc/RingBuffer/ring_buffer.d ./Core/Inc/RingBuffer/ring_buffer.o ./Core/Inc/RingBuffer/ring_buffer.su

.PHONY: clean-Core-2f-Inc-2f-RingBuffer

