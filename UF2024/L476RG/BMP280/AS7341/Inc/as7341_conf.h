/**
 * Private configuration file for the AS7341 library.
 * This example is configured for STM32L4, I2C .
 */

#ifndef __AS7341_CONF_H__
#define __AS7341_CONF_H__

// Choose a microcontroller family
//#define STM32F0
//#define STM32F1
//#define STM32F4
//#define STM32L0
//#define STM32L1
#define STM32L4
//#define STM32F3
//#define STM32H7
//#define STM32F7
//#define STM32G0
//#define STM32C0

// Choose a bus
#define AS7341_USE_I2C
//#define SSD1306_USE_SPI

// I2C Configuration
#define AS7341_I2C_PORT        hi2c1
#define AS7341_I2C_ADDR        (0x39 << 1)



#endif /* __AS7341_CONF_H__ */
