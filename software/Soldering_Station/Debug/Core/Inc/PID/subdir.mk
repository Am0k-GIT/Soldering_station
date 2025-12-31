################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Inc/PID/PID.c 

OBJS += \
./Core/Inc/PID/PID.o 

C_DEPS += \
./Core/Inc/PID/PID.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Inc/PID/%.o Core/Inc/PID/%.su Core/Inc/PID/%.cyclo: ../Core/Inc/PID/%.c Core/Inc/PID/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F401xC -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Inc-2f-PID

clean-Core-2f-Inc-2f-PID:
	-$(RM) ./Core/Inc/PID/PID.cyclo ./Core/Inc/PID/PID.d ./Core/Inc/PID/PID.o ./Core/Inc/PID/PID.su

.PHONY: clean-Core-2f-Inc-2f-PID

