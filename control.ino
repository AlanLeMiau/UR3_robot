/******************************************************************************
 *
 *      ---- VIVOXIE ----
 *
 *  Fecha: 15 - enero - 2020
 *  Desarrolladores:
 *          --> Marco Cardoso
 *          --> Alan Fuentes
 *
 *      Primer test de ROS con Arduino.
 *      Recibe datos de nodo ROS por topics python_imu0 y python_imu1
 *      Controla los motores del brazo robótico 
 *
 *****************************************************************************/
#include "Stepper.h"
#include "Servo5.h"

/***  vars.h  ***/
//#include "vars.h"

/* ROS variables */
// Sustituir código para ROS de roboticArm 
String Data;

// MEGA interruption pins: 2, 3, 18, 19, 20, 21
const uint8_t lim_b0 = 3;
const uint8_t lim_s0 = 18;
const uint8_t lim_s1 = 19;
const uint8_t lim_e0 = 20;
const uint8_t lim_e1 = 21;

/* Interruption variables */
volatile bool stop_base = false;
volatile bool stop_shoulder = false;
// volatile bool stop_shoulder = false;
volatile bool stop_elbow = false;
// volatile bool stop_elbow = false;

// volatile bool dir_bs = false;
// volatile bool dir_sh = false;
// volatile bool dir_el = false;

/* Program constants */
const uint8_t BASE      = 1;
const uint8_t SHOULDER  = 2;
const uint8_t ELBOW     = 3;

const bool    POSITIVE  = true;
const bool    NEGATIVE  = false;

const uint8_t grip_Closed = 150;
const uint8_t grip_Opened = 50;
const uint8_t stepsPerRevolution = 200;
// const uint16_t year = 7700;
// Una vuelta (200 steps) aprox 9.35°
// Un grado aprox 21.38 steps

/* Program variables */
bool    grip    = false;
uint8_t moveReg = 0x00;
/* Movement Register
 * moveReg -> [negative 0 - positive 0]
 * [elbow shoulder base 0 - elbow shoulder base 0]
 */
/*** end vars.h ***/

// Motors instances 
Stepper base(stepsPerRevolution, 22, 23, 24, 25);
Stepper shoulder(stepsPerRevolution, 26, 27, 28, 29);
Stepper elbow(stepsPerRevolution, 30, 31, 32, 33);
Servo5 wrist; //pin 44
Servo5 gripper; // pin 45

void rosFun(void);
void stepOff(void);
void setMovement(int motor, int direction);

void setup()
{
    // Sustituir código para ROS de roboticArm 
    // iniciar nodos y suscribr
    Serial.begin(9600);

    /* Pins setup */    
    wrist.attach(44);
    gripper.attach(45);
   
    pinMode(lim_b0, INPUT_PULLUP); 
    pinMode(lim_s0, INPUT_PULLUP);
    pinMode(lim_s1, INPUT_PULLUP);
    pinMode(lim_e0, INPUT);         // PULLUP by hardware RN1B
    pinMode(lim_e1, INPUT);         // PULLUP by hardware RN1C

    /* Interruptions setup */
    attachInterrupt(digitalPinToInterrupt(lim_b0), check_base, CHANGE);
    attachInterrupt(digitalPinToInterrupt(lim_s0), check_shoulder, CHANGE);
    // attachInterrupt(digitalPinToInterrupt(lim_s1), stop_shoulder, CHANGE);
    attachInterrupt(digitalPinToInterrupt(lim_e0), check_elbow, CHANGE);
    // attachInterrupt(digitalPinToInterrupt(lim_e1), stop_elbow, CHANGE);

    /* INIT */
    stepOff();

    wrist.write(0);
    gripper.write(grip_Closed);

} // end setup

void loop()
{
    // Sustituir código para ROS de roboticArm     
    if (Serial.available() > 0)
    {
        Data = Serial.readString();
    }

    rosFun();

    //delay(50);
} //  end loop()

/* ISR for sensors interruptions */ 
void check_base() {
  // state = !state;
  if(digitalRead(lim_b0) == HIGH){
      stop_base = false;
  }
  if(digitalRead(lim_b0) == LOW){
      stop_base = true;
  }
} // end check_base

void check_shoulder() {
  // state = !state;
  if(digitalRead(lim_s0) == HIGH){
      stop_shoulder = false;
  }
  if(digitalRead(lim_s0) == LOW){
      stop_shoulder = true;
  }
} // end check_shoulder

void check_elbow() {
  // state = !state;
  if(digitalRead(lim_e0) == HIGH){
      stop_elbow = false;
  }
  if(digitalRead(lim_e0) == LOW){
      stop_elbow = true;
  }
} // end check_elbow


void rosFun(void)
{
    /*    
     * if para diferenciar entre imus
     * Resuelto: Tomar de roboticArm cuando se pase a ROS
     * Entrega para cada motor pasos y dirección en que se mueve  
     */

    /**** SIMULANDO ENTRADAS DE ROS ****/
    int    pasos = 200;
    base.direction = 1;
    setMovement(BASE, base.direction);
    

    /********************* MOVING STEP MOTORS *************************/
    // Moving 1 step at same time using stepMotor sequence
    int8_t sec = 0;
    for (sec=0; sec < 4 ; sec++)
    {   
        if ((moveReg & (1<<1)) && !stop_base)
        {
            base.stepMotor(sec);
            //Serial.println("+ base");
        }
        if ((moveReg & (1<<2)) && !stop_shoulder)
        {
            shoulder.stepMotor(sec);
            //Serial.println("+ shoulder");
        }
        if ((moveReg & (1<<3)) && !stop_elbow)
        {
            elbow.stepMotor(sec);
            //Serial.println("+ elbow");
        }
        delay(15);  // Considerar el uso de micros() para los delays 
                    // Ese 15 fue arbitrario pero jaló        
    } // end for positive sequence

    for (sec=3; sec >= 0 ; sec--)
    {   
        if ((moveReg & (1<<5)) && !stop_base)
        {
            base.stepMotor(sec);
            //Serial.println("- base");
        }
        if ((moveReg & (1<<6)) && !stop_shoulder)
        {
            shoulder.stepMotor(sec);
            //Serial.println("- shoulder");
        }
        if ((moveReg & (1<<7)) && !stop_elbow)
        {
            elbow.stepMotor(sec);
            //Serial.println("- elbow");
        }
        delay(15);  // Considerar el uso de micros() para los delays 
                    //Ese 15 fue arbitrario pero jaló        
    } // end for negative sequence

    pasos--; // Hay que contar los pasos
    
    stepOff();

    /********************* MOVING SERVO MOTORS *************************/
    // wrist.write(value)
    //Serial.println("wrist");
    if(grip == true)
    {
        gripper.write(grip_Closed);
    }else{
        gripper.write(grip_Opened);
    }

    return;
    
} // end rosFun

/***
 * stepOff Turn off the motor outputs to avoid heat them
 */
void stepOff(void)
{
    uint8_t i = 22;
    for(i=22 ; i<34 ; i++)
    {
        digitalWrite(i, LOW);
    }
    return;
} // end stepOff

/***
 * setMovement enable the motor movements and set the direction
 */
void setMovement(int motor, int direction)
{
    if (direction == 1)  // true -> Positive movement
    {
        bitWrite(moveReg, motor + 4 , 1);
        bitWrite(moveReg, motor     , 0);
    }
    if (direction == 0) // false -> Negative movement
    {
        bitWrite(moveReg, motor     , 1);
        bitWrite(moveReg, motor + 4 , 0);    
    }
    return;
} // end setMovement
