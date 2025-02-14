/*
 * ________________________________________________________________________________________________________
 * Copyright (c) 2020 GebraBit Inc. All rights reserved.
 *
 * This software, related documentation and any modifications thereto (collectively �Software�) is subject
 * to GebraBit and its licensors' intellectual property rights under U.S. and international copyright
 * and other intellectual property rights laws. 
 *
 * GebraBit and its licensors retain all intellectual property and proprietary rights in and to the Software
 * and any use, reproduction, disclosure or distribution of the Software without an express license agreement
 * from GebraBit is strictly prohibited.
 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT 
 * NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT IN  
 * NO EVENT SHALL GebraBit BE LIABLE FOR ANY DIRECT, SPECIAL, INDIRECT, INCIDENTAL, OR CONSEQUENTIAL DAMAGES, 
 * OR ANY DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT,
 * NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE
 * OF THE SOFTWARE.
 * @Author       	: Sepehr Azimi
 * ________________________________________________________________________________________________________
 */
#include "GebraBit_TSL25721.h"

/*========================================================================================================================================= 
 * @brief     Read  data from  spacial register address.
 * @param     regAddr Register Address of TSL25721 that reading data from this address
 * @param     data    Pointer to Variable that data is saved .
 * @return    None
 ========================================================================================================================================*/
void GB_TSL25721_Read_Reg_Data(uint8_t regAddr,  uint8_t *data)																			/*		Read Burst Data From Register			*/
{
	Wire.beginTransmission(TSL25721_ADDRESS);
    Wire.write(0x80|regAddr); 
    Wire.endTransmission(false); 
    Wire.requestFrom((uint8_t)TSL25721_ADDRESS, (uint8_t)1);
	delay(15);
    if (Wire.available()) {
        *data = Wire.read(); 
    }
}
/*========================================================================================================================================= 
 * @brief     Read multiple data from first spacial register address.
 * @param     regAddr First Register Address of TSL25721 that reading multiple data start from this address
 * @param     data    Pointer to Variable that multiple data is saved .
 * @param     byteQuantity Quantity of data that we want to read .
 * @return    None
 ========================================================================================================================================*/
void GB_TSL25721_Burst_Read(uint8_t regAddr,  uint8_t *data, uint16_t byteQuantity)																			/*		Read Burst Data From Register			*/
{
	Wire.beginTransmission(TSL25721_ADDRESS);
    Wire.write(0x80|0x20|regAddr); 
    Wire.endTransmission(false); 
    Wire.requestFrom((uint8_t)TSL25721_ADDRESS, (uint8_t)byteQuantity); 
	delay(15);
    for (uint16_t i = 0; i < byteQuantity; i++) {
        if (Wire.available()) {
            data[i] = Wire.read();
        }
    }
}
/*========================================================================================================================================= 
 * @brief     Read data from spacial bits of a register.
 * @param     regAddr     Register Address of TSL25721 .
 * @param     start_bit   Start Bit location .(0 to 7)
 * @param     len         Quantity of Bits want to read(1 to 8) 
 * @param     data        Pointer to Variable that register Bits value is saved .
 * @return    status      Return status
 ========================================================================================================================================*/
void GB_TSL25721_Read_Reg_Bits (uint8_t regAddr, uint8_t start_bit, uint8_t len, uint8_t* data)
{
	uint8_t tempData = 0;
	GB_TSL25721_Read_Reg_Data( regAddr, &tempData);
	uint8_t mask = ((1 << len) - 1) << (start_bit - len + 1); //formula for making a broom of 1&0 for gathering desired bits
	tempData &= mask; // zero all non-important bits in data
	tempData >>= (start_bit - len + 1); //shift data to zero position
	*data = tempData;
}

/*========================================================================================================================================= 
 * @brief     Write  data to  spacial register address.
 * @param     regAddr First Register Address of TSL25721 that reading multiple data start from this address
 * @param     data    Variable that to be written .
 * @return    None
 ========================================================================================================================================*/
void GB_TSL25721_Write_Reg_Data(uint8_t regAddr,  uint8_t data)																			/*		Read Burst Data From Register			*/
{
	Wire.beginTransmission(TSL25721_ADDRESS);
    Wire.write(0x80|regAddr); 
    Wire.write(data); 
    Wire.endTransmission();
}
/*========================================================================================================================================= 
 * @brief     Write multiple data from first spacial register address.
 * @param     regAddr First Register Address of TSL25721 that  multiple data to be written start from this address
 * @param     data    Pointer to multiple data Variable that to be written.
 * @param     byteQuantity Quantity of data that we want to Write .
 * @return    None
 ========================================================================================================================================*/
void GB_TSL25721_Burst_Write(uint8_t regAddr,  uint8_t *data, uint16_t byteQuantity)																			/*		Read Burst Data From Register			*/
{
	Wire.beginTransmission(TSL25721_ADDRESS);
    Wire.write(0x80|0x20|regAddr);
	delay(15);
	for (uint16_t i = 0; i < byteQuantity; i++){
		Wire.write(data[i]);
	}
    Wire.endTransmission();
}
/*=========================================================================================================================================
 * @brief     Write data to spacial bits of a register.
 * @param     regAddr     Register Address of TSL25721 .
 * @param     start_bit   Start Bit location .(0 to 7)
 * @param     len         Quantity of Bits want to write(1 to 8) 
 * @param     data        Value that will be writen to register bits .
 * @return    status      Return status
 ========================================================================================================================================*/
void GB_TSL25721_Write_Reg_Bits(uint8_t regAddr, uint8_t start_bit, uint8_t len, uint8_t data)
{
	uint8_t tempData = 0;
	GB_TSL25721_Read_Reg_Data( regAddr, &tempData) ;	
	uint8_t mask = ((1 << len) - 1) << (start_bit - len + 1);
	data <<= (start_bit - len + 1); // shift data into correct position
	data &= mask; // zero all non-important bits in data
	tempData &= ~(mask); // zero all important bits in existing byte
	tempData |= data; // combine data with existing byte
	GB_TSL25721_Write_Reg_Data(regAddr,  tempData);
}

/*=========================================================================================================================================
 * @brief     Enable Or Disable Internal Oscillator
 * @param     TSL25721   TSL25721 Struct OSCILLATOR variable
 * @param     als        Value is from TSL25721_Ability Enume
 * @return    Nothing
 ========================================================================================================================================*/
void GB_TSL25721_Internal_Oscillator ( GebraBit_TSL25721 * TSL25721 , TSL25721_Ability osc ) 
{
 GB_TSL25721_Write_Reg_Bits(TSL25721_ENABLE, START_MSB_BIT_AT_0, BIT_LENGTH_1, osc);
 TSL25721->OSCILLATOR = osc ;
}
/*=========================================================================================================================================
 * @brief     Enable Or Disable ALS
 * @param     TSL25721   TSL25721 Struct ALS variable
 * @param     als        Value is from TSL25721_Ability Enume
 * @return    Nothing
 ========================================================================================================================================*/
void GB_TSL25721_ALS ( GebraBit_TSL25721 * TSL25721 , TSL25721_Ability als ) 
{
 GB_TSL25721_Write_Reg_Bits(TSL25721_ENABLE, START_MSB_BIT_AT_1, BIT_LENGTH_1, als);
 TSL25721->ALS = als ;
}
/*=========================================================================================================================================
 * @brief     Enable Or Disable Interrupt
 * @param     TSL25721   TSL25721 Struct Interrupt variable
 * @param     intr        Value is from TSL25721_Ability Enume
 * @return    Nothing
 ========================================================================================================================================*/
void GB_TSL25721_Interrupt ( GebraBit_TSL25721 * TSL25721 , TSL25721_Ability intr ) 
{
 GB_TSL25721_Write_Reg_Bits(TSL25721_ENABLE, START_MSB_BIT_AT_4, BIT_LENGTH_1, intr);
 TSL25721->INTERRUPT = intr ;
}

/*=========================================================================================================================================
 * @brief     Clear  TSL25721 Interrupt
 * @param     TSL25721   TSL25721 Struct INTERRRUPT_STATUS variable
 * @return    Nothing
 ========================================================================================================================================*/
void GB_TSL25721_Clear_Interrupt ( GebraBit_TSL25721 * TSL25721 ) 
{
	uint8_t TBuff[1];
	TBuff[0]= TSL25721_ALS_INTERRUPT_CLEAR;
	Wire.beginTransmission(TSL25721_ADDRESS); 
    Wire.write(TBuff[0]); 
    Wire.endTransmission();
	TSL25721->INTERRRUPT_STATUS = INTERRUPT_INACTIVE ;
}

/*=========================================================================================================================================
 * @brief     Enable Or Disable Sleep After Interrupt
 * @param     TSL25721   TSL25721 Struct SLEEP_AFTER_INTERRUPT variable
 * @param     intafs        Value is from TSL25721_Ability Enume
 * @return    Nothing
 ========================================================================================================================================*/
void GB_TSL25721_Sleep_After_Interrupt ( GebraBit_TSL25721 * TSL25721 , TSL25721_Ability intafs ) 
{
 GB_TSL25721_Write_Reg_Bits(TSL25721_ENABLE, START_MSB_BIT_AT_6, BIT_LENGTH_1, intafs);
 TSL25721->SLEEP_AFTER_INTERRUPT = intafs ;
}
/*=========================================================================================================================================
 * @brief     Enable Or Disable Sleep Wait Timer
 * @param     TSL25721   TSL25721 Struct WAIT_TIMER variable
 * @param     timer        Value is from TSL25721_Ability Enume
 * @return    Nothing
 ========================================================================================================================================*/
void GB_TSL25721_Wait_Timer ( GebraBit_TSL25721 * TSL25721 , TSL25721_Ability timer ) 
{
 GB_TSL25721_Write_Reg_Bits(TSL25721_ENABLE, START_MSB_BIT_AT_3, BIT_LENGTH_1, timer);
 TSL25721->WAIT_TIMER = timer ;
}
/*=========================================================================================================================================
 * @brief     Enable Or Disable WAIT_LONG_12X
 * @param     TSL25721   TSL25721 Struct WAIT_LONG_12X variable
 * @param     wlong        Value is from TSL25721_Ability Enume
 * @return    Nothing
 ========================================================================================================================================*/
void GB_TSL25721_Wait_Long_12x ( GebraBit_TSL25721 * TSL25721 , TSL25721_Ability wlong ) 
{
 GB_TSL25721_Write_Reg_Bits(TSL25721_CONFIG, START_MSB_BIT_AT_1, BIT_LENGTH_1, wlong);
 TSL25721->WAIT_LONG_12X = wlong ;
}
/*=========================================================================================================================================
 * @brief     Check Wait Long 12x Status
 * @param     TSL25721   TSL25721 Struct WAIT_LONG_12X variable
 * @return    Nothing
 ========================================================================================================================================*/
void GB_TSL25721_Check_Wait_Long_12x ( GebraBit_TSL25721 * TSL25721  ) 
{
	uint8_t wlong;
 	GB_TSL25721_Read_Reg_Bits(TSL25721_CONFIG, START_MSB_BIT_AT_1, BIT_LENGTH_1, (uint8_t*)&TSL25721->WAIT_LONG_12X);
}
/*=========================================================================================================================================
 * @brief     Set Wait Time
 * @param     TSL25721   TSL25721 Struct WAIT_TIME_mS variable
 * @param     wait        Value for Wait Time
 * @return    Nothing
 ========================================================================================================================================*/
void GB_TSL25721_Wait_Time ( GebraBit_TSL25721 * TSL25721 , float wait  )  
{
 GB_TSL25721_Check_Wait_Long_12x ( TSL25721  ) ;
 if(TSL25721->WAIT_LONG_12X  ==  Enable)
  TSL25721->WAIT_TIME_STEP = 32.8 ;
 else if ( TSL25721->WAIT_LONG_12X  ==  Disable )
  TSL25721->WAIT_TIME_STEP = 2.73 ;
 TSL25721->WAIT_TIME_mS = wait ;
 TSL25721->WAIT_TIME = 256 - (TSL25721->WAIT_TIME_mS / TSL25721->WAIT_TIME_STEP ); 
 GB_TSL25721_Write_Reg_Data(TSL25721_WTIME,TSL25721->WAIT_TIME);
}
/*=========================================================================================================================================
 * @brief     Set Integration Time
 * @param     TSL25721   TSL25721 Struct INTEGRATION_TIME_mS variable
 * @param     time        Value for Integration Time
 * @return    Nothing
 ========================================================================================================================================*/
void GB_TSL25721_Integration_Time ( GebraBit_TSL25721 * TSL25721 ,float time )  
{
 TSL25721->INTEGRATION_TIME_STEP = 2.73 ;
 TSL25721->INTEGRATION_TIME_mS = time ;
 TSL25721->INTEGRATION_TIME = 256 - (TSL25721->INTEGRATION_TIME_mS / TSL25721->INTEGRATION_TIME_STEP );
 GB_TSL25721_Write_Reg_Data(TSL25721_ATIME, TSL25721->INTEGRATION_TIME);
}
/*=========================================================================================================================================
 * @brief     Enable Or Disable 0.16 Scale
 * @param     TSL25721   TSL25721 Struct ALS_GAIN_0P16_SCALE variable
 * @param     scale        Value is from TSL25721_Ability Enume
 * @return    Nothing
 ========================================================================================================================================*/
void GB_TSL25721_ALS_Gain_0p16_Scale ( GebraBit_TSL25721 * TSL25721 , TSL25721_Ability scale ) 
{
 GB_TSL25721_Write_Reg_Bits(TSL25721_CONFIG, START_MSB_BIT_AT_2, BIT_LENGTH_1, scale);
 TSL25721->ALS_GAIN_0P16_SCALE = scale ;
}
/*=========================================================================================================================================
 * @brief     Check 0.16 Scale Status
 * @param     TSL25721   TSL25721 Struct ALS_GAIN_0P16_SCALE variable
 * @return    Nothing
 ========================================================================================================================================*/
void GB_TSL25721_Check_ALS_Gain_0p16_Scale( GebraBit_TSL25721 * TSL25721  ) 
{
 GB_TSL25721_Read_Reg_Bits(TSL25721_CONFIG, START_MSB_BIT_AT_2, BIT_LENGTH_1, (uint8_t*)&TSL25721->ALS_GAIN_0P16_SCALE);
}

/*=========================================================================================================================================
 * @brief     Set ALS Gain
 * @param     TSL25721   TSL25721 Struct ALS_GAIN variable & ALS_GAIN_VALUE variable
 * @param     gain        Value is from TSL25721_ALS_Gain Enume
 * @return    Nothing
 ========================================================================================================================================*/
void GB_TSL25721_ALS_Gain ( GebraBit_TSL25721 * TSL25721 , TSL25721_ALS_Gain gain ) 
{
 GB_TSL25721_Write_Reg_Bits(TSL25721_CONTROL, START_MSB_BIT_AT_1, BIT_LENGTH_2, gain);
 TSL25721->ALS_GAIN = gain ;
 	switch(TSL25721->ALS_GAIN)
	 {
	  case ALS_GAIN_1X:
		TSL25721->ALS_GAIN_VALUE = 1 ;
    break;
		case ALS_GAIN_8X:
		TSL25721->ALS_GAIN_VALUE = 8 ;
    break;	
		case ALS_GAIN_16X:
		TSL25721->ALS_GAIN_VALUE = 16 ;
    break;
		case ALS_GAIN_120X:
		TSL25721->ALS_GAIN_VALUE = 120 ;
    break;			
	 }
}
/*
M403Z 
*/
/*=========================================================================================================================================
 * @brief     Read TSL25721 Part ID
 * @param     TSL25721   TSL25721 Struct PART_ID variable
 * @return    Nothing
 ========================================================================================================================================*/
void GB_TSL25721_Read_Part_ID ( GebraBit_TSL25721 * TSL25721  ) 
{
 GB_TSL25721_Read_Reg_Data(TSL25721_ID, &TSL25721->PART_ID);
}
/*=========================================================================================================================================
 * @brief     Read TSL25721 STATUS
 * @param     TSL25721   TSL25721 Struct DATA variable  & INTERRRUPT_STATUS variable
 * @return    Nothing
 ========================================================================================================================================*/
void GB_TSL25721_Read_STATUS ( GebraBit_TSL25721 * TSL25721 ) 
{
 GB_TSL25721_Read_Reg_Data(TSL25721_STATUS, &TSL25721->STATUS_VALUE);
 TSL25721->INTERRRUPT_STATUS = (TSL25721->STATUS_VALUE & 0x10)>>4  ;
 TSL25721->DATA = TSL25721->STATUS_VALUE & 0x01  ;
}
/*=========================================================================================================================================
 * @brief     Select Interrupt Persist
 * @param     TSL25721   TSL25721 Struct INTERRUPT_PERSIST variable
 * @param     persist    Value is from TSL25721_Interrupt_Persist Enume
 * @return    Nothing
 ========================================================================================================================================*/
void GB_TSL25721_Interrupt_Persistence ( GebraBit_TSL25721 * TSL25721 , TSL25721_Interrupt_Persistence persist ) 
{
 GB_TSL25721_Write_Reg_Bits(TSL25721_PERS, START_MSB_BIT_AT_3, BIT_LENGTH_4, persist);
 TSL25721->INTERRUPT_PERSISTENCE = persist ;
}
/*=========================================================================================================================================
 * @brief     Set Interrupt Low Threshold
 * @param     LTR303ALS   LTR303ALS Struct INTERRUPT_LOW_THRESHOLD variable
 * @return    Nothing
 ========================================================================================================================================*/
void GB_TSL25721_Interrupt_Low_Threshold ( GebraBit_TSL25721 * TSL25721 , uint16_t limit ) 
{
 GB_TSL25721_Write_Reg_Data(TSL25721_AILTH, (uint8_t)limit>>8);
 GB_TSL25721_Write_Reg_Data(TSL25721_AILTL, (uint8_t)(limit&0xFF));
 TSL25721->INTERRUPT_LOW_THRESHOLD = limit ;
}
/*=========================================================================================================================================
 * @brief     Set Interrupt High Threshold
 * @param     LTR303ALS   LTR303ALS Struct INTERRUPT_HIGH_THRESHOLD variable
 * @return    Nothing
 ========================================================================================================================================*/
void GB_TSL25721_Interrupt_High_Threshold ( GebraBit_TSL25721 * TSL25721 , uint16_t limit ) 
{
 GB_TSL25721_Write_Reg_Data(TSL25721_AIHTH, (uint8_t)limit>>8);
 GB_TSL25721_Write_Reg_Data(TSL25721_AIHTL, (uint8_t)(limit&0xFF));
 TSL25721->INTERRUPT_HIGH_THRESHOLD = limit ;
}
/*=========================================================================================================================================
 * @brief     initialize TSL25721
 * @param     TSL25721     TSL25721 Struct 
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_TSL25721_initialize( GebraBit_TSL25721 * TSL25721 )
{
	GB_TSL25721_Read_Part_ID(TSL25721);
	GB_TSL25721_Read_STATUS(TSL25721);
	GB_TSL25721_Internal_Oscillator(TSL25721,Enable);
	GB_TSL25721_Interrupt_Low_Threshold(TSL25721,0);
	GB_TSL25721_Interrupt_High_Threshold(TSL25721,0);
	GB_TSL25721_Interrupt_Persistence(TSL25721,CONSECUTIVE_10_ALS_VALUE_OUT_OF_THR_RANGE);
	GB_TSL25721_Sleep_After_Interrupt(TSL25721,Disable);
	GB_TSL25721_Interrupt(TSL25721,Disable);

}
/*=========================================================================================================================================
 * @brief     Configure TSL25721
 * @param     TSL25721  Configure TSL25721 
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_TSL25721_Configuration(GebraBit_TSL25721 * TSL25721)
{
	GB_TSL25721_Wait_Long_12x(TSL25721,Disable);
	GB_TSL25721_Wait_Time(TSL25721,200);
	GB_TSL25721_Wait_Timer(TSL25721,Enable);
	GB_TSL25721_ALS_Gain_0p16_Scale(TSL25721,Disable);
	GB_TSL25721_ALS_Gain(TSL25721,ALS_GAIN_8X);
	GB_TSL25721_Integration_Time(TSL25721,100);
	GB_TSL25721_ALS(TSL25721,Enable);

}
/*=========================================================================================================================================
 * @brief     Get Raw Data Of Channel 0 & Channel 1
 * @param     LTR303ALS  store Raw Data Of Channel 0 & Channel 1 in GebraBit_LTR303ALS Struct ALS_DATA_CH1 & ALS_DATA_CH0
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_TSL25721_Read_CH0_CH1_Raw_Data(GebraBit_TSL25721 * TSL25721)
{
	GB_TSL25721_Read_STATUS( TSL25721 );
	if (TSL25721->DATA == DATA_IS_VALID) 
	{    
   GB_TSL25721_Burst_Read( TSL25721_C0DATA , TSL25721->ADC_DATA , 4);
	 TSL25721->ALS_DATA_CH0 = ((uint16_t)TSL25721->ADC_DATA[1]<<8)|((uint16_t)TSL25721->ADC_DATA[0])  ;
   TSL25721->ALS_DATA_CH1 = ((uint16_t)TSL25721->ADC_DATA[3]<<8)|((uint16_t)TSL25721->ADC_DATA[2])  ;
	}
} 
/*=========================================================================================================================================
 * @brief     Calculate Light intensity Lux
 * @param     GebraBit_LTR303ALS Struct COUNTER_PER_LUX & ALS_LUX 
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_TSL25721_Lux_Reading(GebraBit_TSL25721 * TSL25721)
{
	float lux1=0;
	float lux2=0;
  TSL25721->COUNTER_PER_LUX = (TSL25721->INTEGRATION_TIME_mS * TSL25721->ALS_GAIN_VALUE)/(1 * 60.0f) ; ////// 1 Beacuse in open air
  lux1 = (TSL25721->ALS_DATA_CH0 - (1.87f * TSL25721->ALS_DATA_CH1))/TSL25721->COUNTER_PER_LUX;
	lux2 = ((0.63f * TSL25721->ALS_DATA_CH0) - TSL25721->ALS_DATA_CH1)/TSL25721->COUNTER_PER_LUX;
	if ( (lux1 <= 0) && (lux2 <= 0))
		TSL25721->ALS_LUX = 0 ;
	else if ( lux1 >= lux2 )
		TSL25721->ALS_LUX = lux1 ;
	else if ( lux2 >= lux1 )
		TSL25721->ALS_LUX = lux2 ;
} 
/*=========================================================================================================================================
 * @brief     Get Data  
 * @param     TSL25721       GebraBit_TSL25721 Staruct
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_TSL25721_Get_Data(GebraBit_TSL25721 * TSL25721)
{
  GB_TSL25721_Read_CH0_CH1_Raw_Data( TSL25721 );
	GB_TSL25721_Lux_Reading(TSL25721);
}
/*----------------------------------------------------------------------------------------------------------------------------------------*
 *                                                                      End                                                               *
 *----------------------------------------------------------------------------------------------------------------------------------------*/
//	  GB_TSL25721_Write_Command( TSL25721_ID);
//		HAL_I2C_Master_Transmit(TSL25721_I2C,TSL25721_WRITE_ADDRESS,TSL25721_ID,1,100);
//	  HAL_I2C_Master_Receive(TSL25721_I2C ,  TSL25721_READ_ADDRESS, data,3, 100);
//		HAL_I2C_Master_Receive(TSL25721_I2C ,  TSL25721_READ_ADDRESS, &TSL25721_Module.Register_Cache,1, 100);
//    GB_TSL25721_Read_Reg_Data (TSL25721_ID,&TSL25721_Module.Register_Cache);
//		GB_TSL25721_Write_Reg_Data(TSL25721_AIHTL, 0x0C)	;
//		GB_TSL25721_Write_Reg_Data(TSL25721_AIHTH, 0x0D)	;
//		GB_TSL25721_Burst_Read    ( TSL25721_AIHTL , TSL25721_Module.REGISTER_DATA , 2);
//		GB_TSL25721_Burst_Write   ( TSL25721_AIHTL , data , 2);
//		GB_TSL25721_Burst_Read    ( TSL25721_AIHTL , TSL25721_Module.REGISTER_DATA , 2);
//		GB_TSL25721_Write_Reg_Data(TSL25721_AIHTL, 0x0)	;
//		GB_TSL25721_Write_Reg_Data(TSL25721_AIHTH, 0x0)	;
//		GB_TSL25721_Burst_Read    ( TSL25721_AIHTL , TSL25721_Module.REGISTER_DATA , 2);