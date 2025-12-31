################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Inc/BUTTONS/buttons.c 

OBJS += \
./Core/Inc/BUTTONS/buttons.o 

C_DEPS += \
./Core/Inc/BUTTONS/buttons.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Inc/BUTTONS/%.o Core/Inc/BUTTONS/%.su Core/Inc/BUTTONS/%.cyclo: ../Core/Inc/BUTTONS/%.c Core/Inc/BUTTONS/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -DUSE_HAL_DRIVER -DSTM32F401xC -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Inc-2f-BUTTONS

clean-Core-2f-Inc-2f-BUTTONS:
	-$(RM) ./Core/Inc/BUTTONS/buttons.cyclo ./Core/Inc/BUTTONS/buttons.d ./Core/Inc/BUTTONS/buttons.o ./Core/Inc/BUTTONS/buttons.su

.PHONY: clean-Core-2f-Inc-2f-BUTTONS

