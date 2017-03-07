#ifndef PID_v1_h
#define PID_v1_h
#define LIBRARY_VERSION	1.1.1

#include <stdint.h>
#include <stdbool.h>
#include <avr/io.h>
#include <assert.h>
#include <stdio.h>

//Constants used in some of the functions below
#define AUTOMATIC	1
#define MANUAL	0
#define DIRECT  0
#define REVERSE  1

//commonly used functions **************************************************************************
PID_Setup(float, float, float, int, int64_t);  // * constructor. links the PID to the Input, Output, and
//   Set point.  Initial tuning parameters are also set here

void PID_SetMode(int Mode);               // * sets PID to either Manual (0) or Auto (non-0)

float PID_Compute(float, float, int64_t);                       // * performs the PID calculation.  it should be
//   called every time loop() cycles. ON/OFF and
//   calculation frequency can be set using SetMode
//   SetSampleTime respectively

void PID_SetOutputLimits(float, float); //clamps the output to a specific range. 0-255 by default, but
//it's likely the user will want to change this depending on
//the application



//available but not commonly used functions ********************************************************
void PID_SetTunings(float, float,       // * While most users will set the tunings once in the
float);         	  //   constructor, this function gives the user the option
//   of changing tunings during runtime for Adaptive control
void PID_SetControllerDirection(int);	  // * Sets the Direction, or "Action" of the controller. DIRECT
//   means the output will increase when error is positive. REVERSE
//   means the opposite.  it's very unlikely that this will be needed
//   once it is set in the constructor.
void PID_SetSampleTime(int);              // * sets the frequency, in Milliseconds, with which
//   the PID calculation is performed.  default is 100



//Display functions ****************************************************************
float PID_GetKp();						  // These functions query the pid for internal values.
float PID_GetKi();						  //  they were created mainly for the pid front-end,
float PID_GetKd();						  // where it's important to know what is actually
int PID_GetMode();						  //  inside the PID.
int PID_GetDirection();					  //


void PID_Initialize();

float dispKp;				// * we'll hold on to the tuning parameters in user-entered
float dispKi;				//   format for display purposes
float dispKd;				//

float kp;                  // * (P)roportional Tuning Parameter
float ki;                  // * (I)ntegral Tuning Parameter
float kd;                  // * (D)erivative Tuning Parameter

int controllerDirection;

float myInput;              // * Pointers to the Input, Output, and Setpoint variables
float myOutput;             //   This creates a hard link between the variables and the
float mySetpoint;           //   PID, freeing the user from having to constantly tell us
//   what these values are.  with pointers we'll just know.

unsigned long lastTime;
float ITerm, dInput, PTerm, lastInput;

unsigned long SampleTime;
float outMin, outMax;
bool inAuto;
#endif