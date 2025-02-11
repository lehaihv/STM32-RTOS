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
    u_int8_t regData;
    status = AS7341_ReadRegister(AS7341_WHOAMI,&regData);
    errNum += (status != HAL_OK);

    if (regData != AS7341_CHIP_ID) {
        return 255;
    }   
}

bool setASTEP(uint16_t astep_value){

}
//
bool setATIME(uint8_t atime_value){

}
//
bool setGain(as7341_gain_t gain_value){

}
//
uint16_t getASTEP(){

}
//
uint8_t getATIME();
as7341_gain_t getGain();

long getTINT();
float toBasicCounts(uint16_t raw);

bool readAllChannels(void);
bool readAllChannels(uint16_t *readings_buffer);
void delayForData(int waitTime = 0);
    
uint16_t readChannel(as7341_adc_channel_t channel);
    
uint16_t getChannel(as7341_color_channel_t channel);

bool startReading(void);
bool checkReadingProgress();
bool getAllChannels(uint16_t *readings_buffer);

uint16_t detectFlickerHz(void);

void setup_F1F4_Clear_NIR(void);
void setup_F5F8_Clear_NIR(void);

void powerEnable(bool enable_power);
bool enableSpectralMeasurement(bool enable_measurement);

bool setHighThreshold(uint16_t high_threshold);
bool setLowThreshold(uint16_t low_threshold);

uint16_t getHighThreshold(void);
uint16_t getLowThreshold(void);

bool enableSpectralInterrupt(bool enable_int);
bool enableSystemInterrupt(bool enable_int);

    bool setAPERS(as7341_int_cycle_count_t cycle_count);
    bool setSpectralThresholdChannel(as7341_adc_channel_t channel);

    uint8_t getInterruptStatus(void);
    bool clearInterruptStatus(void);

    bool spectralInterruptTriggered(void);
    uint8_t spectralInterruptSource(void);
    bool spectralLowTriggered(void);
    bool spectralHighTriggered(void);

    bool enableLED(bool enable_led);
    bool setLEDCurrent(uint16_t led_current_ma);
    uint16_t getLEDCurrent(void);

    void disableAll(void);

    bool getIsDataReady();
    bool setBank(bool low); // low true gives access to 0x60 to 0x74

    as7341_gpio_dir_t getGPIODirection(void);
    bool setGPIODirection(as7341_gpio_dir_t gpio_direction);
    bool getGPIOInverted(void);
    bool setGPIOInverted(bool gpio_inverted);
    bool getGPIOValue(void);
    bool setGPIOValue(bool);

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


