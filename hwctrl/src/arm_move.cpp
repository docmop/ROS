#include "ros/ros.h"
#include "hwctrl/arm_move.h"

// gcc -o test robotic_arm.c -lpigpio -lpthread -lrt
//Web Adresse Pigpio Library: http://abyz.co.uk/rpi/pigpio/

#include <pigpio.h>
#include <stdio.h>
void Servo(float);
void LA_Drive(int handle);

//ADC addresses
#define ADS7830_I2C_Addr 0x48
#define ADS7830_I2C_Read 0x91

//Channel 1
#define ADS7830_CH0 0x08

//PDMode
#define ADS7830_PDMode0 0x00    //Powerdown between conversions
#define ADS7830_PDMode1 0x04    //Ref OFF/ADC ON
#define ADS7830_PDMode2 0x08    //Ref ON/ADC OFF
#define ADS7830_PDMode3 0x0C    //Ref ON/ADC ON

#define length_extended 0x05
#define length_retracted 0xE5

//Hardware PWM
#define PWM_0_gpio18 18					//PWM 0 GPIO18
#define PWM_0_gpio12 12					//PWM 0 GPIO12
#define PWM_1_gpio13 13					//PWM 1 GPIO13
#define PWM_1_gpio19 19					//PWM 1 GPIO19
#define PWM_Duty_Driver 1000000		//Dutycycle 100% = 1000000
#define PWM_Freq_Servo 50			//PWM Frequenz Servo 50Hz
#define PWM_Freq_Driver 1000			//PWM Frequenz DC Motor Driver 200Hz
#define PWM_Duty_Off 0				//PWM Dutycycle off
#define PWM_Freq_Off 0				//PWM Frequenz off



bool move(hwctrl::arm_move::Request  &req,
	  hwctrl::arm_move::Response &res);

void LA_Drive(int handle, int linear_pos);
int Drive_UP ();
int Drive_Down ();
void Servo(float AD_Soll);



int main(int argc, char **argv)
{
	ros::init(argc, argv, "arm_server");
	ros::NodeHandle n;
	
	ros::ServiceServer service = n.advertiseService("arm", move);
	ROS_INFO("Ready to move arm.");
	
	ros::spin();
	
	return 0;
} 

bool move(hwctrl::arm_move::Request  &req,
	  hwctrl::arm_move::Response &res)
{
	ROS_INFO("request: linear_pos=%d, servo_pos=%d", (int)req.linear_pos, (int)req.servo_pos);
	
	// Tobias Schommer --------------------------------------------------------------------------------------
	
	int handle;
	int AD_Value_CH0 = 0;
	int AD_Dummy = 0;
	
	if(gpioInitialise()<0)
		return 1;
	handle = i2cOpen(1, ADS7830_I2C_Addr, 0);
	
	i2cOpen(1, ADS7830_I2C_Addr, 0);
	i2cWriteByte(handle,(ADS7830_CH0 << 4) | ADS7830_PDMode3); //Dummy senden
	i2cWriteByte(handle,(ADS7830_CH0 << 4) | ADS7830_PDMode3); //
	AD_Value_CH0 = i2cReadByte(handle);
	printf("ADC_Wert: %d\n",(int)AD_Value_CH0);
	
	LA_Drive(handle, req.linear_pos);
	
	i2cClose(handle);
	
	gpioTerminate();
	
	// Tobias Schommer --------------------------------------------------------------------------------------
	
	ROS_INFO("done: %d", (int)res.done);
	return true;
}



// Tobias Schommer --------------------------------------------------------------------------------------

void LA_Drive(int handle, int linear_pos)
{
	int AD_Soll = 0;
	int AD_Value_CH0 = i2cReadByte(handle);
	
	AD_Soll = ((linear_pos * 224) / 300) + 4; // AD_Soll berechnet sich aus der Hoehe die dem Programm(in mm) vorgegeben wird.
	// 4 dazu addieren da AD_Werte erst bei 4 beginnen
	
	
	if((AD_Soll < AD_Value_CH0)) //Wenn AD_Soll < AD_Value_CH0 => fahre runter
	{
		ROS_DEBUG("AD_Soll: %d\n", (int)AD_Soll);
		Drive_Down();
		
		
		while(AD_Soll < AD_Value_CH0)
		{
			i2cWriteByte(handle,(ADS7830_CH0 << 4) | ADS7830_PDMode3); //Funktioniert so
			AD_Value_CH0 = i2cReadByte(handle);
			ROS_DEBUG("ADC_Wert1: %d\n",(int)AD_Value_CH0);
			gpioDelay(1000);
		}
		ROS_DEBUG("Unten erreicht\n");
		gpioHardwarePWM(PWM_0_gpio18, PWM_Freq_Off, PWM_Duty_Off);
// 		Servo(AD_Soll);
		ROS_DEBUG("ADC_Ende: %d\n",(int)AD_Value_CH0);
	}
	
	else if(AD_Soll > AD_Value_CH0) // Wenn AD_Soll > AD_Value_CH0 => fahre hoch
		
	{
		ROS_DEBUG("AD_Soll: %d\n", (int)AD_Soll);
		Drive_UP();
		
		while(AD_Soll > AD_Value_CH0)
		{
			i2cWriteByte(handle,(ADS7830_CH0 << 4) | ADS7830_PDMode3);
			AD_Value_CH0 = i2cReadByte(handle);
			ROS_DEBUG("ADC_Wert1: %d\n",(int)AD_Value_CH0);
			gpioDelay(1000);
		}
		ROS_DEBUG("Oben erreicht\n");
		gpioHardwarePWM(PWM_0_gpio18, PWM_Freq_Off, PWM_Duty_Off);
// 		Servo(AD_Soll);
		ROS_DEBUG("ADC_Ende: %d\n",(int)AD_Value_CH0);
	}
	
	else if(AD_Soll == AD_Value_CH0)						//Wenn AD_Soll schon erreicht => Nur Servo fahren
		
	{
		ROS_DEBUG("Target height already reached");
// 		Servo(AD_Soll);
	}
	
}

int Drive_UP ()
{
	gpioWrite(22, 1);  //gpio 22 high --> Inv 
	gpioHardwarePWM(PWM_0_gpio18, PWM_Freq_Driver, PWM_Duty_Driver ); //Dutycycle 100%, Frequenz 1000 Hz
	
	ROS_DEBUG("Hochfahren\n");
	return 0;
}


int Drive_Down ()

{
	gpioWrite(22, 0);  //gpio 22 low --> Inv
	gpioHardwarePWM(PWM_0_gpio18, PWM_Freq_Driver, PWM_Duty_Driver ); //Dutycycle 100%, Frequenz 1000 Hz
	
	ROS_DEBUG("Runterfahren\n");
	
	return 0;
}


void Servo(float AD_Soll)
{
	float Servo_Value_Gpio13 = 0;
	float Servo_Value_Gpio18 = 0;
	
	ROS_DEBUG("AD_Soll: %d\n", (int)AD_Soll);
	Servo_Value_Gpio18 =  ((1000.0f/224.0f)*((float)(AD_Soll -5)))+1000.0f;
	ROS_DEBUG("Servowert: %d\n",(int)Servo_Value_Gpio18);
	gpioServo(17, (int)Servo_Value_Gpio18);
	gpioDelay(100000);	//Delay to wait for motor to finish moving
	
	/*
	 * //HardwarePWM
	 * Servo_Value_Gpio13 = 50000.0f + ((50000.0f/224.0f)*((float)(AD_Soll -4))); //Berechnung Duty Cycle Servo
	 * gpioHardwarePWM(PWM_1_gpio13, PWM_Freq_Servo, (int)Servo_Value_Gpio13); //Hardware PWM
	 * gpioDelay(1000);
	 */
}
