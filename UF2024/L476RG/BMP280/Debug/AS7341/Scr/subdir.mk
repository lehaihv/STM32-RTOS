################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../AS7341/Scr/as7341.c 

OBJS += \
./AS7341/Scr/as7341.o 

C_DEPS += \
./AS7341/Scr/as7341.d 


# Each subdirectory must supply rules for building sources it contributes
AS7341/Scr/%.o AS7341/Scr/%.su AS7341/Scr/%.cyclo: ../AS7341/Scr/%.c AS7341/Scr/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L476xx -c -I../Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -I"/Users/macbookpro5530/Documents/GitHub/STM32-RTOS/UF2024/L476RG/BMP280/OLED/Inc" -I"/Users/macbookpro5530/Documents/GitHub/STM32-RTOS/UF2024/L476RG/BMP280/AS7341/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-AS7341-2f-Scr

clean-AS7341-2f-Scr:
	-$(RM) ./AS7341/Scr/as7341.cyclo ./AS7341/Scr/as7341.d ./AS7341/Scr/as7341.o ./AS7341/Scr/as7341.su

.PHONY: clean-AS7341-2f-Scr

