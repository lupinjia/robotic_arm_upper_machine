/* FILE:    HCPCA9685.cpp
   DATE:    10/06/16
   VERSION: 0.1
   AUTHOR:  Andrew Davies

   Library created by Hobby Components Ltd (HOBBYCOMPONENTS.COM)
   
10/06/16 version 0.1: Original version

Arduino Library for the PCA9685 16 Channel 12-bit PWM Servo Motor Driver.   
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



#include "HCPCA9685.h"
#include <Wire.h>


/* Constructor for the library */
HCPCA9685::HCPCA9685(byte I2CAdd)
{
	_I2CAdd = I2CAdd;
}


/* Initialises the I2C interface and puts the device in to a default state */
void HCPCA9685::Init(boolean Mode)
{
	Wire.begin();
	
	/* If not already, put device to sleep. This is required to change the Prescaller value */
	Sleep(true);
	
	/* Set register addressing mode to auto increment to speed up writing to 		 multiple registers */
	_AutoIncrement(true);
	
	/* If the library has been initialised for driving servos set the period
	   frequency (prescaller) for 50Hz (2ms) period. */
	if(Mode == SERVO_MODE)
	{
		OutputOnTime(0, 0);
		OutputOffTime(0, 0);
		SetPeriodFreq(SERVO);
	}
}



/* Sets the sleep state of the device where:

   Mode is the required state. Valid values are:
		true 	(puts the device to sleep)
		false 	(brings the device out of sleep mode)
   
   If the device is woken up from a previously set sleep state then this function will wait 500us for the oscillator to stabilise. */
void HCPCA9685::Sleep(boolean Mode)
{
  byte Data = I2CReadReg(MODE1);

  Data = (Data & MODE1_SLEEP_MASK) | (Mode << MODE1_SLEEP_BIT);
  I2CWriteReg(MODE1, Data);

  /* Check if device was put in to sleep mode and if so give the oscillator time to restart */
  if(Data & ~MODE1_RESTART_MASK)
  {
    delayMicroseconds(500);
    Data |= (1 << MODE1_RESTART_BIT);
  }
}



/* Sets the period frequency (prescaller) in Hz where:
   Freq is the required frequency of on cycle. Valid values are:
		24 (24Hz) to 1526 (1.526KHz)
   
   These values are based on the devices internal 25MHz oscillator (default). 
   If using an external clock you will need to adjust the value for OSC_FREQ found in
   the HCPCA9685.h header file */
void HCPCA9685::SetPeriodFreq(unsigned int Freq)
{
  byte Data = (OSC_FREQ / (4096L * Freq)) - 1;
  I2CWriteReg(PRE_SCALE, Data);
  Serial.println(Data);
}



/* Sets the value for the prescaller where:
   Period is the required period for one cycle. Valid values are:
		0x03 to 0xFF 
   
   The prescaller value can be determined with the following formula:
   prescale value = (25MHz / (4096 * period frequency in Hz)) - 1 */
void HCPCA9685::SetPreScaller(byte Period)
{
  I2CWriteReg(PRE_SCALE, Period);
}



/* When the library has been initiliased to servo mode this function sets the
   on time for the servo where:
   
   Chan is the PWM output pin to change. Valid values are
		0 (PWM output pin 0) to 15 (PWM output pin 15)
		
   Pos is the required position of the servo. Valid values are
		0 (servo fully anti-clockwise) to 480 (servo fully clockwise)
		
	The maximum value is based in the default servo SERVO_TRIM_MAX and SERVO_TRIM_MIN trim settings found in the HCPCA9685.h header file.
	These default to 590 for SERVO_TRIM_MAX and 110 for SERVO_TRIM_MIN.
	Thay can be adjusted to match the minimum and maximum end-stop positions for your servo(s). If these values are changed then the new max value for Pos can 
	be calculated as (SERVO_TRIM_MAX - SERVO_TRIM_MIN) */
void HCPCA9685::Servo(byte Chan, unsigned int Pos)
{
  //unsigned int Offset = (unsigned int)Chan << 8;
  Pos += SERVO_TRIM_MIN;
  if (Pos > SERVO_TRIM_MAX)
    Pos = SERVO_TRIM_MAX;
  Output(Chan, 0 /*Offset*/, Pos /*+ Offset*/);
}




/* Sets the turn ON time and OFF time for one of the 15 PWM outputs where:
   
   Chan is the PWM output pin to change. Valid values are
		0 (PWM output pin 0) to 15 (PWM output pin 15)
		
   On_Time is the point at which to turn the output ON within the cycle period. Valid values are
		0 to 4095 (decimal)

    Off_Time is the point at which to turn the output OFF within the cycle period. Valid values are
		0 to 4095 (decimal)
*/
void HCPCA9685::Output(byte Chan, unsigned int On_Time, unsigned int Off_Time)
{
  if(Chan > 15)
    Chan = 15;

  Chan = LED0_ON_L + (Chan << 2);
    
  Wire.beginTransmission(_I2CAdd); 
  Wire.write(Chan);
  Wire.write(On_Time & 0xFF); 
  Wire.write(On_Time >> 8);
  Wire.write(Off_Time & 0xFF); 
  Wire.write(Off_Time >> 8);
  Wire.endTransmission(true);
}




/* Sets the turn ON time and OFF time for ALL of the 15 PWM outputs where:

   On_Time is the point at which to turn the outputs ON within the cycle period. Valid values are
		0 to 4095 (decimal)

    Off_Time is the point at which to turn the outputs OFF within the cycle period. Valid values are
		0 to 4095 (decimal) */
void HCPCA9685::Output(unsigned int On_Time, unsigned int Off_Time)
{
  Wire.beginTransmission(_I2CAdd); 
  Wire.write(ALL_LED_ON_L);
  Wire.write(On_Time & 0xFF); 
  Wire.write(On_Time >> 8);
  Wire.write(Off_Time & 0xFF); 
  Wire.write(Off_Time >> 8);
  Wire.endTransmission(true);
}




/* Sets the turn ON time for one of the 15 PWM outputs where:
   
   Chan is the PWM output pin to change. Valid values are
		0 (PWM output pin 0) to 15 (PWM output pin 15)
		
   Time is the point at which to turn the output on within the cycle period. Valid values are
		0 to 4095 (decimal) */
void HCPCA9685::OutputOnTime(byte Chan, unsigned int Time)
{
  if(Chan > 15)
    Chan = 15;
  Chan = LED0_ON_L + (Chan << 2);

  Wire.beginTransmission(_I2CAdd); 
  Wire.write(Chan);
  Wire.write(Time & 0xFF); 
  Wire.write(Time >> 8);
  Wire.endTransmission(true);
}




/* Sets the turn OFF time for one of the 15 PWM outputs where:
   
   Chan is the PWM output channel pin to change. Valid values are
		0 (PWM output pin 0) to 15 (PWM output pin 15)
		
   Time is the point at which to turn the output off within the cycle period. Valid values are
		0 to 4095 (decimal) */
void HCPCA9685::OutputOffTime(byte Chan, unsigned int Time)
{
  if(Chan > 15)
    Chan = 15;
  Chan = LED0_OFF_L + (Chan << 2);

  Wire.beginTransmission(_I2CAdd); 
  Wire.write(Chan);
  Wire.write(Time & 0xFF); 
  Wire.write(Time >> 8);
  Wire.endTransmission(true);
}




/* Sets the state of the 16 PWM outputs when the OE pin is driven high where:
	
	State is the required state of the output pins. Valid values are:
		OUTNE_LOW				Output pins are pulled low when OE = 1
		OUTNE_HIGH 				Output pins are pulled high when OE = 1
		OUTNE_HIGH_IMPEDANCE 	Output pins are high-impedance when OE = 1 */
void HCPCA9685::OutputNotEnableState(byte State)
{
  byte Data = I2CReadReg(MODE2);
  Data = (Data & MODE2_OUTNE_MASK) | (State << MODE2_OUTNE_BIT);
  I2CWriteReg(MODE2, Data);
}



/* Sets the driver type for the PWM outputs where:

	Mode is the required type. Valid values are:
		OUTDRV_OPEN_DRAIN	The 16 PWM outputs are configured with an open-drain structure.
		OUTDRV_TOTEM_POLE	The 16 PWM outputs are configured with a totem pole structure. */
void HCPCA9685::OutputDrivers(boolean Mode)
{
  byte Data = I2CReadReg(MODE2);
  Data = (Data & MODE2_OUTDRV_MASK) | (Mode << MODE2_OUTDRV_BIT);
  I2CWriteReg(MODE2, Data);
}



/* Sets the point at which the 16 PWM outputs change state where:
	
	Mode is the required state. Valid values are:
		OCH_STOP		Outputs change on I2C STOP command.
		OCH_ACK			Outputs change on I2C ACK. */
void HCPCA9685::OCH(boolean Mode)
{
  byte Data = I2CReadReg(MODE2);
  Data = (Data & MODE2_OCH_MASK) | (Mode << MODE2_OCH_BIT);
  I2CWriteReg(MODE2, Data);
}



/* Inverts the state of the PWM output pins where:
	
	Mode is the require state. Valid values are:
		false	The PWM output pins are not inverted.
		true	The PWM output pins are inverted. */
void HCPCA9685::Invert(boolean Mode)
{
  byte Data = I2CReadReg(MODE2);
  Data = (Data & MODE2_INVRT_MASK) | (Mode << MODE2_INVRT_BIT);
  I2CWriteReg(MODE2, Data);
}



/* Sets whether or not the device will respond the I2C sub address 1 where:

	Mode sets the required state. Valid values are:
		false		The device will not respond to I2C sub address 1.
		true		The device will respond to I2C sub address 1. */
void HCPCA9685::Enable_Sub1(boolean Mode)
{
  byte Data = I2CReadReg(MODE1);
  Data = (Data & MODE1_SUB1_MASK) | (Mode << MODE1_SUB1_BIT);
  I2CWriteReg(MODE1, Data);
}



/* Sets whether or not the device will respond the I2C sub address 2 where:

	Mode sets the required state. Valid values are:
		false		The device will not respond to I2C sub address 2.
		true		The device will respond to I2C sub address 2. */
void HCPCA9685::Enable_Sub2(boolean Mode)
{
  byte Data = I2CReadReg(MODE1);
  Data = (Data & MODE1_SUB2_MASK) | (Mode << MODE1_SUB2_BIT);
  I2CWriteReg(MODE1, Data);
}



/* Sets whether or not the device will respond the I2C sub address 3 where:

	Mode sets the required state. Valid values are:
		false		The device will not respond to I2C sub address 3.
		true		The device will respond to I2C sub address 3. */
void HCPCA9685::Enable_Sub3(boolean Mode)
{
  byte Data = I2CReadReg(MODE1);
  Data = (Data & MODE1_SUB3_MASK) | (Mode << MODE1_SUB3_BIT);
  I2CWriteReg(MODE1, Data);
}



/* Sets whether or not the device will respond the I2C ALLCALL address where:

	Mode sets the required state. Valid values are:
		false		The device will not respond to I2C ALLCALL address.
		true		The device will respond to I2C ALLCALL address. */
void HCPCA9685::Enable_AllCall(boolean Mode)
{
  byte Data = I2CReadReg(MODE1);
  Data = (Data & MODE1_ALLCALL_MASK) | (Mode << MODE1_ALLCALL_BIT);
  I2CWriteReg(MODE1, Data);
}



/* Sets the I2C address for one of the 3 sub addresses where:
	
	SubAddress is one of the 3 sub addresses to set the I2C address for. Valid values are:
		SUBADR1		Selects sub address 1.
		SUBADR2		Selects sub address 2.
		SUBADR3		Selects sub address 3.
		
	Address is and 8 bit byte representing the I2C address to set the selected sub address too. Only the 7 LSBs representing the I2C-bus sub address are valid. */
void HCPCA9685::SetSubAddress(byte SubAddress, byte Address)
{
  I2CWriteReg(SubAddress, ((0x7F & Address) << 1));
}




/* Sets the I2C address for the ALLCALL address where:
		
	Address is and 8 bit byte representing the I2C address to set the ALLCALL  address too. Only the 7 LSBs representing the I2C-bus ALLCALL address are valid. */
void HCPCA9685::SetAllCallAddress(byte Address)
{
  I2CWriteReg(ALLCALLADR, ((0x7F & Address) << 1));
}




/* Sets external clock mode allowing the device to be driven by an external clock source. When in external clock mode, it can only be cleared by recycling the power */
void HCPCA9685::ExtClk(void)
{
  Sleep(true);
  
  byte Data = I2CReadReg(MODE1);
  Data = Data | (1 << MODE1_EXTCLK_BIT) | (1 << MODE1_SLEEP_BIT);
  I2CWriteReg(MODE1, Data);
}




/* Writes a byte of data to one of the devices registers where:

	Register is the address of the register to write to.
	
	Data is the 8 bit data to write to the register */
void HCPCA9685::I2CWriteReg(byte Register, byte Data)
{
  Wire.beginTransmission(_I2CAdd); 
  Wire.write(Register);
  Wire.write(Data); 
  Wire.endTransmission(true);
}



/* Reads one of the devices registers where:

	Register is the address of the register to read from.
	
	Returns an 8 bit byte containing the contents of the specified register */
byte HCPCA9685::I2CReadReg(byte Register)
{
  byte Data;
  Wire.beginTransmission(_I2CAdd); 
  Wire.write(Register);
  Wire.endTransmission(true);
  
  Wire.requestFrom(_I2CAdd, 1);
  Data = Wire.read();
  Wire.endTransmission(); 
  
  return Data;
}



/* Enables the devices register address auto increment mode */ 
void HCPCA9685::_AutoIncrement(boolean Mode)
{
  byte Data = I2CReadReg(MODE1);
  Data = (Data & MODE1_AI_MASK) | (Mode << MODE1_AI_BIT);
  I2CWriteReg(MODE1, Data);
}