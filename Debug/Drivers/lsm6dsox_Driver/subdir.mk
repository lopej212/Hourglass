################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/lsm6dsox_Driver/lsm6dsox_reg.c 

OBJS += \
./Drivers/lsm6dsox_Driver/lsm6dsox_reg.o 

C_DEPS += \
./Drivers/lsm6dsox_Driver/lsm6dsox_reg.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/lsm6dsox_Driver/%.o Drivers/lsm6dsox_Driver/%.su Drivers/lsm6dsox_Driver/%.cyclo: ../Drivers/lsm6dsox_Driver/%.c Drivers/lsm6dsox_Driver/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L432xx -c -I../Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -I../Drivers/lsm6dsox_Driver -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-lsm6dsox_Driver

clean-Drivers-2f-lsm6dsox_Driver:
	-$(RM) ./Drivers/lsm6dsox_Driver/lsm6dsox_reg.cyclo ./Drivers/lsm6dsox_Driver/lsm6dsox_reg.d ./Drivers/lsm6dsox_Driver/lsm6dsox_reg.o ./Drivers/lsm6dsox_Driver/lsm6dsox_reg.su

.PHONY: clean-Drivers-2f-lsm6dsox_Driver

