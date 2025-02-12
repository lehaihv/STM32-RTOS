################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../OLED/Scr/ssd1306.c \
../OLED/Scr/ssd1306_fonts.c \
../OLED/Scr/ssd1306_tests.c 

OBJS += \
./OLED/Scr/ssd1306.o \
./OLED/Scr/ssd1306_fonts.o \
./OLED/Scr/ssd1306_tests.o 

C_DEPS += \
./OLED/Scr/ssd1306.d \
./OLED/Scr/ssd1306_fonts.d \
./OLED/Scr/ssd1306_tests.d 


# Each subdirectory must supply rules for building sources it contributes
OLED/Scr/%.o OLED/Scr/%.su OLED/Scr/%.cyclo: ../OLED/Scr/%.c OLED/Scr/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L476xx -c -I../Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -I"/Users/macbookpro5530/Documents/GitHub/STM32-RTOS/UF2024/L476RG/BMP280/OLED/Inc" -I"/Users/macbookpro5530/Documents/GitHub/STM32-RTOS/UF2024/L476RG/BMP280/AS7341/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-OLED-2f-Scr

clean-OLED-2f-Scr:
	-$(RM) ./OLED/Scr/ssd1306.cyclo ./OLED/Scr/ssd1306.d ./OLED/Scr/ssd1306.o ./OLED/Scr/ssd1306.su ./OLED/Scr/ssd1306_fonts.cyclo ./OLED/Scr/ssd1306_fonts.d ./OLED/Scr/ssd1306_fonts.o ./OLED/Scr/ssd1306_fonts.su ./OLED/Scr/ssd1306_tests.cyclo ./OLED/Scr/ssd1306_tests.d ./OLED/Scr/ssd1306_tests.o ./OLED/Scr/ssd1306_tests.su

.PHONY: clean-OLED-2f-Scr

