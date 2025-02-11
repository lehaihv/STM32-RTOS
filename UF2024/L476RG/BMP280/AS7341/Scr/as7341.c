#include "as7341.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>  // For memcpy


/* Initialize the oled screen */
uint8_t AS7341_Init(void) {
    // Errors detected and status
    uint8_t errNum = 0;
    HAL_StatusTypeDef status;

    // check device id from WHOAMI
    uint8_t regData;
    status = AS7341_ReadRegister(AS7341_WHOAMI, &regData);
    errNum += (status != HAL_OK);

    if (regData != AS7341_CHIP_ID) {
        errNum = 255; //return 255;
    }
    return errNum;
}

// Low level functions
// Write register
HAL_StatusTypeDef AS7341_WriteRegister(uint8_t reg, uint8_t *data) {
    return HAL_I2C_Mem_Write(&AS7341_I2C_PORT, AS7341_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, 1, HAL_MAX_DELAY);
}

// Read register
HAL_StatusTypeDef AS7341_ReadRegister(uint8_t reg, uint8_t *data) {
    return HAL_I2C_Mem_Write(&AS7341_I2C_PORT, AS7341_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, 1, HAL_MAX_DELAY);
}

// Read multiple register
HAL_StatusTypeDef AS7341_ReadRegisters(uint8_t reg, uint8_t *data, uint8_t length) {
    return HAL_I2C_Mem_Write(&AS7341_I2C_PORT, AS7341_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, length, HAL_MAX_DELAY);
}


