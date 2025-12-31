################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Inc/24CXX/ee24.c 

OBJS += \
./Core/Inc/24CXX/ee24.o 

C_DEPS += \
./Core/Inc/24CXX/ee24.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Inc/24CXX/%.o Core/Inc/24CXX/%.su Core/Inc/24CXX/%.cyclo: ../Core/Inc/24CXX/%.c Core/Inc/24CXX/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F401xC -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Inc-2f-24CXX

clean-Core-2f-Inc-2f-24CXX:
	-$(RM) ./Core/Inc/24CXX/ee24.cyclo ./Core/Inc/24CXX/ee24.d ./Core/Inc/24CXX/ee24.o ./Core/Inc/24CXX/ee24.su

.PHONY: clean-Core-2f-Inc-2f-24CXX

