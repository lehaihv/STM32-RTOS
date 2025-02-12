#include "as7341.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>  // For memcpy


/* Initialize the AS7341 */
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

/* Setup operating parameters of the AS7341 */
void AS7341_Setup(void) {

	uint8_t regData;

	// Set ENABLE Register: Spectral Measurement Enable; Power ON.
	regData = 0x03;
	AS7341_WriteRegister(AS7341_ENABLE, &regData);

	// Set CONFIG Register
	regData = 0x00;
	AS7341_WriteRegister(AS7341_CONFIG, &regData);

	// Set ATIME Register: Sets the number of integration steps from 1 to 256 ASTEP x (n+1).
	regData = 0x00;
	AS7341_WriteRegister(AS7341_ATIME, &regData);

	// Set ASTEP_L Register: 03E7 (999): 2.78ms
	regData = 0xE7;
	AS7341_WriteRegister(AS7341_ASTEP_L, &regData);

	// Set ASTEP_H Register
	regData = 0x03;
	AS7341_WriteRegister(AS7341_ASTEP_H, &regData);

	// Set FD_GAIN Register 256x
	regData = 0x24;
	AS7341_WriteRegister(AS7341_FD_TIME2, &regData);

}
// Low level functions
// Write register
HAL_StatusTypeDef AS7341_WriteRegister(uint8_t reg, uint8_t *data) {
    return HAL_I2C_Mem_Write(&AS7341_I2C_PORT, AS7341_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, 1, 10000);//HAL_MAX_DELAY);
}

// Read register
HAL_StatusTypeDef AS7341_ReadRegister(uint8_t reg, uint8_t *data) {
    return HAL_I2C_Mem_Read(&AS7341_I2C_PORT, AS7341_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, 1, 10000);//HAL_MAX_DELAY);
}

// Read multiple register
HAL_StatusTypeDef AS7341_ReadRegisters(uint8_t reg, uint8_t *data, uint8_t length) {
    return HAL_I2C_Mem_Read(&AS7341_I2C_PORT, AS7341_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, length, 10000);//HAL_MAX_DELAY);
}


