/* FILE:    HCPCA9685.h
   DATE:    10/06/16
   VERSION: 0.1
   AUTHOR:  Andrew Davies

   Library created by Hobby Components Ltd (HOBBYCOMPONENTS.COM)
   
10/06/16 version 0.1: Original version

Arduino Library header for the PCA9685 16 Channel 12-bit PWM Servo Motor Driver.   
Currently supported products:

16 Channel 12-bit PWM Servo Motor Driver Module (HCMODU0097) available from hobbycomponents.com 


You may copy, alter and reuse this code in any way you like, but please leave
reference to HobbyComponents.com in your comments if you redistribute this code.
This software may not be used directly for the purpose of selling products that
directly compete with Hobby Components Ltd's own range of products.

THIS SOFTWARE IS PROVIDED "AS IS". HOBBY COMPONENTS MAKES NO WARRANTIES, WHETHER
EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE, ACCURACY OR LACK OF NEGLIGENCE.
HOBBY COMPONENTS SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR ANY DAMAGES,
INCLUDING, BUT NOT LIMITED TO, SPECIAL, INCIDENTAL OR CONSEQUENTIAL DAMAGES FOR ANY
REASON WHATSOEVER.
*/


#ifndef HCPCA9685_h
#define HCPCA9685_h

#include "Arduino.h"


/******************************************************************************
 *							       USER SETTINGS 							  *
 *****************************************************************************/ 

/* The following two values set the minimum and maximum trim values for the 
   servo(s). You can adjust these values if your servo(s) are either hitting 
   their end-stops, or are not able to rotate to the min or max ends of the 
   servo. */
   
#define SERVO_TRIM_MIN 110
#define SERVO_TRIM_MAX 590
 
 
/* If using an external clock change this value to match the clocks frequency 
   in Hz */
#define OSC_FREQ 25000000L

 /****************************************************************************/ 


 /* Register address and bit positions/masks for the devices internal registers */
#define MODE1 0x00
#define MODE1_ALLCALL_BIT 0
#define MODE1_ALLCALL_MASK ~(1 << MODE1_ALLCALL_BIT)
#define MODE1_SUB3_BIT 1
#define MODE1_SUB3_MASK ~(1 << MODE1_SUB3_BIT)
#define MODE1_SUB2_BIT 2
#define MODE1_SUB2_MASK ~(1 << MODE1_SUB2_BIT)
#define MODE1_SUB1_BIT 3
#define MODE1_SUB1_MASK ~(1 << MODE1_SUB1_BIT)
#define MODE1_SLEEP_BIT 4
#define MODE1_SLEEP_MASK ~(1 << MODE1_SLEEP_BIT)
#define MODE1_AI_BIT 5
#define MODE1_AI_MASK ~(1 << MODE1_AI_BIT)
#define MODE1_EXTCLK_BIT 6
#define MODE1_EXTCLK_MASK ~(1 << MODE1_EXTCLK_BIT)
#define MODE1_RESTART_BIT 7
#define MODE1_RESTART_MASK ~(1 << MODE1_RESTART_BIT)

#define MODE2 0x01
#define MODE2_OUTNE_BIT 0
#define MODE2_OUTNE_MASK ~(3 << MODE2_OUTNE_BIT)
#define MODE2_OUTDRV_BIT 2
#define MODE2_OUTDRV_MASK ~(1 << MODE2_OUTDRV_BIT)
#define MODE2_OCH_BIT 3
#define MODE2_OCH_MASK ~(1 << MODE2_OCH_BIT)
#define MODE2_INVRT_BIT 4
#define MODE2_INVRT_MASK ~(1 << MODE2_INVRT_BIT)
#define OCH_ACK 1
#define OCH_STOP 0
#define OUTDRV_TOTEM_POLE 1
#define OUTDRV_OPEN_DRAIN 1
#define OUTNE_LOW 0
#define OUTNE_HIGH 1
#define OUTNE_HIGH_IMPEDANCE 2

#define SUBADR1 0x02
#define SUBADR2 0x03
#define SUBADR3 0x04
#define ALLCALLADR 0x05
#define LED0_ON_L 0x06
#define LED0_ON_H 0x07
#define LED0_OFF_L 0x08
#define LED0_OFF_H 0x09
#define ALL_LED_ON_L 0xFA
#define ALL_LED_ON_H 0xFB
#define ALL_LED_OFF_L 0xFC
#define ALL_LED_OFF_H 0xFD
#define PRE_SCALE 0xFE

#define SERVO 50
#define DEFAULT_MODE false
#define SERVO_MODE true


class HCPCA9685
{
	public:
	HCPCA9685(byte I2C_Add);
	void Init(boolean Mode = DEFAULT_MODE);
	void Sleep(boolean Mode);
	
	void SetPeriodFreq(unsigned int Freq);
	void SetPreScaller(byte Period);
	
	void Servo(byte Chan, unsigned int Pos);
	void Output(unsigned int On_Time, unsigned int Off_Time);
	void Output(byte Chan, unsigned int On_Time, unsigned int Off_Time);
	void OutputOnTime(byte Chan, unsigned int Time);
	void OutputOffTime(byte Chan, unsigned int Time);
	
	void OutputNotEnableState(byte State);
	void OutputDrivers(boolean Mode);
	void OCH(boolean Mode);
	void Invert(boolean Mode);
	void Enable_Sub1(boolean Mode);
	void Enable_Sub2(boolean Mode);
	void Enable_Sub3(boolean Mode);
	void Enable_AllCall(boolean Mode);
	void SetSubAddress(byte SubAddress, byte Address);
	void SetAllCallAddress(byte Address);
	void ExtClk(void);
	
	void I2CWriteReg(byte Register, byte Data);
	byte I2CReadReg(byte Register);	
		
	private:
	int _I2CAdd;
	void _AutoIncrement(boolean Mode);
};
#endif