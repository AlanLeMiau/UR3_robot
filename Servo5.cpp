/******************************************************************************
 *
 *      ---- VIVOXIE ----
 *
 *  Fecha: 15 - enero - 2020
 *  Desarrolladores:
 *          --> Alan Fuentes
 *	servo5.h Servo library for Wiring/Arduino - Version 1.1.0
 *  Only supports ArduinoMEGA
 *	Modifies Timer5 on ATMEGA2560 to generates servo signals 
 *	on pins 44, 45 or 46 with values between the following limits:
 *	Servo limits:
 * 	  0° -> 0.481 ms = 15
 *  180° -> 2.528 ms = 79
 *  On setup must call first servo5_init()
 *  Then you must attach some a pin to Servo
 *  (only pin 44, 45 and 46 available) 
 *	To move a Servo, use the function Write(value)
 *****************************************************************************/
#include <Arduino.h>
#include <avr/io.h>
#include "Servo5.h"

Servo5::Servo5()
{ 
	_NOP();
} // end constructor

/*******************************************************************************
 * attach.
 * Attach and initialize the pin to the servo control signal as Output
 * Parameters: 
 *      servo_pin is the pin where the servo is connected
 *      Only pins 44, 45 or 46 for this library
* Return: error code, 0 if the attach was well done
 *      ERROR 1: No valid pin, must be only 44, 45 or 46
 ******************************************************************************/
uint8_t Servo5::attach(uint8_t servo_pin)
{
	if((servo_pin > 46) | (servo_pin < 44))
	{
		return 1; // ERROR 1: No valid pin, must be only 44, 45 or 46
	}
	else
	{
	    /* Changing PWM frequency to pins 44, 45 and 46 to:
	     * f_PWM = 61.035 Hz
	     * f_PWM =	 f_i/o 		f_i/o:	Clock
	     *		  ------------- 	N:	PreScaler
	     *		   2 * N * TOP    TOP:	Resolution
	     */
	    // f_i/o = 16 MHz clock
	    // WGM5=0010 -> PWM resolution to 9 bits
	    bitClear(TCCR5A, WGM50);
	    bitSet(TCCR5A, WGM51);
	    bitClear(TCCR5B, WGM52);
	    bitClear(TCCR5B, WGM53);
	    // CS5=100 -> PreScaler to 256
	    bitClear(TCCR5B, CS50);
	    bitClear(TCCR5B, CS51);
	    bitSet(TCCR5B, CS52);

		// pin for motor control connection
		this->servo_pin = servo_pin;

		// setup the pins on the microcontroller:
		pinMode(this->servo_pin, OUTPUT);

		return 0;
	}
} // end attach

/*******************************************************************************
 * write.
 * Generates the correct servo control signal to move the servo
 * Parameters: 
 *      value is an angle in degrees
 *      must be between 0 and 180
 * Return: error code, 0 if the signal was well done
 *      ERROR 2: Value out of range, must be between 0° and 180°
 *		ERROR 3: Conversion out of range, must be between 15 and 79
 ******************************************************************************/
uint8_t Servo5::write(uint16_t value)
{
	// first verifies that value is in range
	if((value > 180) | (value < 0))
	{
		return 2; // ERROR 2: Value out of range, must be between 0° and 180°
	}
	else
	{
		// Conversion: degrees to PWM value	
		value = value * 16 / 45;
		value += 15;
		// Checking the value is in range of servo control signal
		if ((value > 79) | (value < 15))
		{
			return 3; // ERROR 3: Conversion out of range, must be between 15 and 79
		}
		else // sending control signal to servo 
		{
			analogWrite(this->servo_pin, value);
			return 0;

		} // end if conversion is in range
	} // end if value is between 0° and 180°
} // end write
