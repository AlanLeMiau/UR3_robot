/******************************************************************************
 *
 *      ---- VIVOXIE ----
 *
 *  Fecha: 15 - enero - 2020
 *  Desarrolladores:
 *          --> Alan Fuentes
 *	servo5.h Servo library for Wiring/Arduino - Version 1.1.0
 *  Only supports ArduinoMEGA because the TIMER5 registers are 
 *	modified on ATMEGA2560 to generates servo signals 
 *	on pins 44, 45 or 46 with values between the following limits:
 *	Servo limits:
 * 	  0° -> 0.481 ms = 15
 *  180° -> 2.528 ms = 79
 *  The resolution is 3°
 *  On setup must call first servo5_init()
 *  Then you must attach some a pin to Servo
 *  (only pin 44, 45 and 46 available) 
 *	To move a Servo, use the function Write(value)
 *****************************************************************************/
// ensure this library description is only included once
#ifndef Servo5_h
#define Servo5_h

#ifdef __cplusplus

#include <stdint.h>

#if ARDUINO >= 100
#include <Arduino.h>       // for delayMicroseconds, digitalPinToBitMask, etc
#else
#include "WProgram.h"      // for delayMicroseconds
#include "pins_arduino.h"  // for digitalPinToBitMask, etc
#endif

// includes the names and memory directions of TIMER5 registers
#include <avr/io.h>

class Servo5
{
    public:
        // constructor
        Servo5();

        // Attach the pinto your servo
        uint8_t attach(uint8_t servo_pin);

        // move method:
        uint8_t write(uint16_t value);
	
	private:
        // Pin only can be 44, 45 or 46
        uint8_t servo_pin;      // Servo pin
    
};

#endif // End __cplusplus
#endif // End servo5_h