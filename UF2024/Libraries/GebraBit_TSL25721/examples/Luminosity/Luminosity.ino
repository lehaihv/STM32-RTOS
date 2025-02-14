#include "GebraBit_TSL25721.h"

GebraBit_TSL25721 TSL25721;

void setup() {
    Wire.begin();           // Initialize the I2C bus
    Serial.begin(9600);     // Initialize serial communication for debugging

    GB_TSL25721_initialize(&TSL25721); // Initialize the TSL25721 sensor
    GB_TSL25721_Configuration(&TSL25721); // Configure the TSL25721 sensor
}

void loop() {
    GB_TSL25721_Get_Data(&TSL25721); // Read data from the sensor
    
    Serial.print("luminosity: ");
    Serial.print(TSL25721.ALS_LUX);
    Serial.println(" lx");
    
    delay(2000); // Delay between readings
}
