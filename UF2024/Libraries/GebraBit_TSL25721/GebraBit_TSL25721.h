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
#ifndef	__TSL25721_H__
#define	__TSL25721_H__
#include "arduino.h"
#include "Wire.h"
#include "stdint.h"
#include "stdlib.h"
#include "stdio.h"
#include "string.h"
#include "math.h"
/************************************************
 *              USER REGISTER MAP               *
 ***********************************************/ 
#define TSL25721_ADDRESS 						  0x39		
#define TSL25721_ENABLE  					    0x00
#define TSL25721_ATIME  							0x01
#define TSL25721_WTIME 								0x03
#define TSL25721_AILTL 								0x04
#define TSL25721_AILTH 								0x05
#define TSL25721_AIHTL 								0x06
#define TSL25721_AIHTH 								0x07
#define TSL25721_PERS 								0x0C
#define TSL25721_CONFIG 							0x0D
#define TSL25721_CONTROL 							0x0F
#define TSL25721_ID 									0x12
#define TSL25721_STATUS 							0x13
#define TSL25721_C0DATA 							0x14
#define TSL25721_C0DATAH 							0x15
#define TSL25721_C1DATA 							0x16
#define TSL25721_C1DATAH 							0x17
#define TSL25721_ALS_INTERRUPT_CLEAR  0xE6 
#define GLASS_ATTENUATION             1.0f  ////// 1 Beacuse in open air
/*----------------------------------------------*
 *           USER REGISTER MAP End              *
 *----------------------------------------------*/ 
 /************************************************
 *         MSB Bit Start Location Begin         *
 ***********************************************/ 
#define START_MSB_BIT_AT_0                    0
#define START_MSB_BIT_AT_1                    1
#define START_MSB_BIT_AT_2                    2
#define START_MSB_BIT_AT_3                    3
#define START_MSB_BIT_AT_4                    4
#define START_MSB_BIT_AT_5                    5
#define START_MSB_BIT_AT_6                    6
#define START_MSB_BIT_AT_7                    7
/*----------------------------------------------*
 *        MSB Bit Start Location End            *
 *----------------------------------------------*/ 
/************************************************
 *          Bit Field Length Begin              *
 ***********************************************/ 
#define BIT_LENGTH_1                          1
#define BIT_LENGTH_2                          2
#define BIT_LENGTH_3                          3
#define BIT_LENGTH_4                          4
#define BIT_LENGTH_5                          5
#define BIT_LENGTH_6                          6
#define BIT_LENGTH_7                          7
#define BIT_LENGTH_8                          8
/*----------------------------------------------*
 *          Bit Field Length End                *
 *----------------------------------------------*/
 /************************************************
 *          Register Values Begin                *
 ***********************************************/ 
#define TSL25721_OSR_256_CONVERSION_TIME				 1
#define TSL25721_OSR_512_CONVERSION_TIME				 2
#define TSL25721_OSR_1024_CONVERSION_TIME				 3
#define TSL25721_OSR_2048_CONVERSION_TIME				 5
#define TSL25721_OSR_4096_CONVERSION_TIME				 9
#define TSL25721_OSR_8192_CONVERSION_TIME				 17
#define ADC_DATA_BUFFER_SIZE              	     4
/*----------------------------------------------*
 *           Register Values End                *
 *----------------------------------------------*/
/**************************************************
 *     Values For Disable And Enable Functions    *
 **************************************************/ 
typedef enum Ability
{  
	Disable = 0     ,                      
	Enable     
}TSL25721_Ability; 
/*************************************************
 *           Values For Reset Process             *
 **************************************************/ 
typedef enum 
{  
	FAILED = 0     ,                      
	DONE     
}TSL25721_Reset_Status;
/*************************************************
 *             Values For ALS Mode               *
 **************************************************/ 
typedef enum ALS_Mode 
{  
	STANDBY = 0     ,                      
	ACTIVE     
}TSL25721_ALS_Mode;
/*************************************************
 *             Values For ALS Gain               *
 **************************************************/ 
typedef enum ALS_Gain 
{
  ALS_GAIN_1X   = 0 ,
  ALS_GAIN_8X   = 1 ,
  ALS_GAIN_16X  = 2 ,
  ALS_GAIN_120X = 3
} TSL25721_ALS_Gain;
/*************************************************
 *       Values For Integration Time             *
 **************************************************/ 
typedef enum Integration_Time
{
  _2P73_mS_INTEGRATION_TIME = 0xFF ,
  _27P3_mS_INTEGRATION_TIME = 0xF6 ,
  _101_mS_INTEGRATION_TIME  = 0xDB ,
  _175_mS_INTEGRATION_TIME  = 0xC0 ,
  _699_mS_INTEGRATION_TIME  = 0x00
} TSL25721_Integration_Time;
/*************************************************
 *       Values For Measurement Rate             *
 **************************************************/ 
typedef enum Measurement_Rate
{
  ALS_MEASRATE_50_mS,
  ALS_MEASRATE_100_mS,
  ALS_MEASRATE_200_mS,
  ALS_MEASRATE_500_mS,
  ALS_MEASRATE_1000_mS,
  ALS_MEASRATE_2000_mS,
} TSL25721_Measurement_Rate;
/*************************************************
 *             Values For Data Status            *
 **************************************************/ 
typedef enum Data_Status 
{  
	OLD_DATA = 0     ,                      
	NEW_DATA     
}TSL25721_Data_Status;
/*************************************************
 *           Values For Interrupt Status         *
 **************************************************/ 
typedef enum Interrupt_Status 
{  
	INTERRUPT_INACTIVE = 0     ,                      
	INTERRUPT_ACTIVE     
}TSL25721_Interrupt_Status;
/*************************************************
 *            Values For Data Valid              *
 **************************************************/ 
typedef enum Data_Valid 
{  
	DATA_IS_INVALID = 0     ,                      
	DATA_IS_VALID     
}TSL25721_Data_Valid;
/**************************************************************
 *       						 Values For Interrupt Persist    			    *
 **************************************************************/ 
typedef enum Interrupt_Persist
{
  EVERY_ALS_CYCLE,
  CONSECUTIVE_1_ALS_VALUE_OUT_OF_THR_RANGE,
  CONSECUTIVE_2_ALS_VALUE_OUT_OF_THR_RANGE,
  CONSECUTIVE_3_ALS_VALUE_OUT_OF_THR_RANGE,
  CONSECUTIVE_5_ALS_VALUE_OUT_OF_THR_RANGE,
  CONSECUTIVE_10_ALS_VALUE_OUT_OF_THR_RANGE,
	CONSECUTIVE_15_ALS_VALUE_OUT_OF_THR_RANGE,
	CONSECUTIVE_20_ALS_VALUE_OUT_OF_THR_RANGE,
	CONSECUTIVE_25_ALS_VALUE_OUT_OF_THR_RANGE,
	CONSECUTIVE_30_ALS_VALUE_OUT_OF_THR_RANGE,
	CONSECUTIVE_35_ALS_VALUE_OUT_OF_THR_RANGE,
	CONSECUTIVE_40_ALS_VALUE_OUT_OF_THR_RANGE,
	CONSECUTIVE_45_ALS_VALUE_OUT_OF_THR_RANGE,
	CONSECUTIVE_50_ALS_VALUE_OUT_OF_THR_RANGE,
	CONSECUTIVE_55_ALS_VALUE_OUT_OF_THR_RANGE,
	CONSECUTIVE_60_ALS_VALUE_OUT_OF_THR_RANGE,
} TSL25721_Interrupt_Persistence;

 /*************************************************
 *  Defining TSL25721 Register & Data As Struct   *
 **************************************************/
typedef	struct TSL25721
{
	  uint8_t                       	Register_Cache;
		uint8_t													PART_ID;
		TSL25721_Ability 								ALS;
		TSL25721_Ability                OSCILLATOR;
	  TSL25721_Ability 								WAIT_TIMER;
	  TSL25721_Ability								WAIT_LONG_12X;
	  float														WAIT_TIME_mS;
	  uint8_t													WAIT_TIME;
	  float														WAIT_TIME_STEP;
		TSL25721_Ability                INTERRUPT;
	  TSL25721_Ability			    			SLEEP_AFTER_INTERRUPT;
		float										        INTEGRATION_TIME_mS;
	  uint8_t									        INTEGRATION_TIME;
	  float														INTEGRATION_TIME_STEP;
	  TSL25721_Ability                ALS_GAIN_0P16_SCALE;
	  TSL25721_ALS_Gain               ALS_GAIN;
	  float		 												ALS_GAIN_VALUE;
	  uint8_t												  STATUS_VALUE;
    TSL25721_Interrupt_Status			  INTERRRUPT_STATUS;
    TSL25721_Data_Valid             DATA;
    TSL25721_Interrupt_Persistence  INTERRUPT_PERSISTENCE;
	  uint16_t                        INTERRUPT_LOW_THRESHOLD;
	  uint16_t                        INTERRUPT_HIGH_THRESHOLD;
		uint8_t 												ADC_DATA[ADC_DATA_BUFFER_SIZE];
		uint16_t               					ALS_DATA_CH0;//Reference to uint16_t where visible+IR data will be stored
		uint16_t               					ALS_DATA_CH1;//Reference to uint16_t where IR-only data will be stored
		float      											COUNTER_PER_LUX;
		double 													ALS_LUX;
}GebraBit_TSL25721;
/*
:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/********************************************************
 *  Declare Read&Write TSL25721 Register Values Functions *
 ********************************************************/
extern void GB_TSL25721_Read_Reg_Data(uint8_t regAddr,  uint8_t *data)	;
extern void GB_TSL25721_Burst_Read(uint8_t regAddr,  uint8_t *data, uint16_t byteQuantity);
extern void GB_TSL25721_Read_Reg_Bits (uint8_t regAddr, uint8_t start_bit, uint8_t len, uint8_t* data);	
extern void GB_TSL25721_Write_Command( uint8_t cmd);
extern void GB_TSL25721_Write_Reg_Data(uint8_t regAddr,  uint8_t data)	;
extern void GB_TSL25721_Burst_Write(uint8_t regAddr,  uint8_t *data, uint16_t byteQuantity)								;
extern void GB_TSL25721_Write_Reg_Bits(uint8_t regAddr, uint8_t start_bit, uint8_t len, uint8_t data);
/********************************************************
 *       Declare TSL25721 Configuration Functions         *
 ********************************************************/
extern void GB_TSL25721_Internal_Oscillator ( GebraBit_TSL25721 * TSL25721 , TSL25721_Ability osc )  ;
extern void GB_TSL25721_ALS ( GebraBit_TSL25721 * TSL25721 , TSL25721_Ability als )  ;
extern void GB_TSL25721_Interrupt ( GebraBit_TSL25721 * TSL25721 , TSL25721_Ability intr )  ;
extern void GB_TSL25721_Clear_Interrupt ( GebraBit_TSL25721 * TSL25721 )  ;
extern void GB_TSL25721_Sleep_After_Interrupt ( GebraBit_TSL25721 * TSL25721 , TSL25721_Ability intafs )  ;
extern void GB_TSL25721_Wait_Timer ( GebraBit_TSL25721 * TSL25721 , TSL25721_Ability timer ) ;
extern void GB_TSL25721_Integration_Time  (GebraBit_TSL25721 * TSL25721 ,float time ) ;
extern void GB_TSL25721_Wait_Long_12x ( GebraBit_TSL25721 * TSL25721 , TSL25721_Ability wlong )  ;
extern void GB_TSL25721_Check_Wait_Long_12x ( GebraBit_TSL25721 * TSL25721  )   ;
extern void GB_TSL25721_Set_Wait_Time ( GebraBit_TSL25721 * TSL25721 , float wait  );
extern void GB_TSL25721_ALS_Gain_0p16_Scale ( GebraBit_TSL25721 * TSL25721 , TSL25721_Ability scale )  ;
extern void GB_TSL25721_Check_ALS_Gain_0p16_Scale( GebraBit_TSL25721 * TSL25721  ) ;
extern void GB_TSL25721_ALS_Gain ( GebraBit_TSL25721 * TSL25721 , TSL25721_ALS_Gain gain ); 
extern void GB_TSL25721_Read_Part_ID ( GebraBit_TSL25721 * TSL25721  ); 
extern void GB_TSL25721_Read_STATUS ( GebraBit_TSL25721 * TSL25721 ) ;
extern void GB_TSL25721_Interrupt_Persistence ( GebraBit_TSL25721 * TSL25721 , TSL25721_Interrupt_Persistence persist ) ;
extern void GB_TSL25721_Interrupt_Upper_Limitation ( GebraBit_TSL25721 * TSL25721 , uint16_t limit );
extern void GB_TSL25721_Interrupt_Lower_Limitation ( GebraBit_TSL25721 * TSL25721 , uint16_t limit )  ;
extern void GB_TSL25721_initialize( GebraBit_TSL25721 * TSL25721 )  ;
extern void GB_TSL25721_Configuration(GebraBit_TSL25721 * TSL25721)  ;
extern void GB_TSL25721_Read_CH0_CH1_Raw_Data(GebraBit_TSL25721 * TSL25721);
extern void GB_TSL25721_Lux_Reading(GebraBit_TSL25721 * TSL25721);
extern void GB_TSL25721_Get_Data(GebraBit_TSL25721 * TSL25721);
#endif
