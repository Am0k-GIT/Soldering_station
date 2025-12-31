################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Inc/PID_/pid.c 

OBJS += \
./Core/Inc/PID_/pid.o 

C_DEPS += \
./Core/Inc/PID_/pid.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Inc/PID_/%.o Core/Inc/PID_/%.su Core/Inc/PID_/%.cyclo: ../Core/Inc/PID_/%.c Core/Inc/PID_/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F401xC -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Inc-2f-PID_

clean-Core-2f-Inc-2f-PID_:
	-$(RM) ./Core/Inc/PID_/pid.cyclo ./Core/Inc/PID_/pid.d ./Core/Inc/PID_/pid.o ./Core/Inc/PID_/pid.su

.PHONY: clean-Core-2f-Inc-2f-PID_

