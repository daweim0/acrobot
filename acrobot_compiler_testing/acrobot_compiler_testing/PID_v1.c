/**********************************************************************************************
* Arduino PID Library - Version 1.1.1
* by Brett Beauregard <br3ttb@gmail.com> brettbeauregard.com
*
* This Library is licensed under a GPLv3 License
**********************************************************************************************/


#include "PID_v1.h"

/*Constructor (...)*********************************************************
*    The parameters specified here are those for for which we can't set up
*    reliable defaults, so we need to have the user set them.
***************************************************************************/
PID_Setup(float Kp, float Ki, float Kd, int ControllerDirection, int64_t mill_time)
{
	mySetpoint = 0;
	inAuto = true;
	
	PID_SetOutputLimits(-126, 126);				//default output limit corresponds to
	//the arduino pwm limits

	SampleTime = 100;							//default Controller Sample Time is 0.1 seconds

	PID_SetControllerDirection(ControllerDirection);
	PID_SetTunings(Kp, Ki, Kd);

	lastTime = mill_time-SampleTime;
}


/* Compute() **********************************************************************
*     This, as they say, is where the magic happens.  this function should be called
*   every time "void loop()" executes.  the function will decide for itself whether a new
*   pid Output needs to be computed.  returns true when the output is computed,
*   false when nothing has been done.
**********************************************************************************/
float PID_Compute(float input, float setPoint, int64_t mill_time)
{
	if(!inAuto) return false;
	unsigned long now = mill_time;
	float timeChange = (float) (now - lastTime) / 16000000.0;
	
	mySetpoint = setPoint;
	
	/*Compute all the working error variables*/
	float error = mySetpoint - input;
	PTerm = kp * error;
	if(timeChange < 0.5) {  // don't use the i term if dt > 0.5 seconds
		ITerm+= ki * error * timeChange;
	}
	if(ITerm > outMax) ITerm= outMax;
	else if(ITerm < outMin) ITerm= outMin;
	ITerm = ITerm * (1 - .005 * timeChange);  // makes the I term slowly decrease over time
	dInput = kd * (input - lastInput) / timeChange;
	
	/*Compute PID Output*/
	float output = PTerm + ITerm- dInput;
	
	if(output > outMax) output = outMax;
	else if(output < outMin) output = outMin;
	
	/*Remember some variables for next time*/
	lastInput = input;
	lastTime = now;
	return output;
}


/* SetTunings(...)*************************************************************
* This function allows the controller's dynamic performance to be adjusted.
* it's called automatically from the constructor, but tunings can also
* be adjusted on the fly during normal operation
******************************************************************************/
void PID_SetTunings(float Kp, float Ki, float Kd)
{
	if (Kp<0 || Ki<0 || Kd<0) return;
	
	dispKp = Kp; dispKi = Ki; dispKd = Kd;
	
	float SampleTimeInSec = ((float)SampleTime)/1000;
	kp = Kp;
	ki = Ki;
	kd = Kd;
	
	if(controllerDirection ==REVERSE)
	{
		kp = (0 - kp);
		ki = (0 - ki);
		kd = (0 - kd);
	}
}

/* SetSampleTime(...) *********************************************************
* sets the period, in Milliseconds, at which the calculation is performed
******************************************************************************/
void PID_SetSampleTime(int NewSampleTime)
{

}

/* SetOutputLimits(...)****************************************************
*     This function will be used far more often than SetInputLimits.  while
*  the input to the controller will generally be in the 0-1023 range (which is
*  the default already,)  the output will be a little different.  maybe they'll
*  be doing a time window and will need 0-8000 or something.  or maybe they'll
*  want to clamp it from 0-125.  who knows.  at any rate, that can all be done
*  here.
**************************************************************************/
void PID_SetOutputLimits(float Min, float Max)
{
	if(Min >= Max) return;
	outMin = Min;
	outMax = Max;
}

/* SetMode(...)****************************************************************
* Allows the controller Mode to be set to manual (0) or Automatic (non-zero)
* when the transition from manual to auto occurs, the controller is
* automatically initialized
******************************************************************************/
void PID_SetMode(int Mode)
{
	bool newAuto = (Mode == AUTOMATIC);
	if(newAuto == !inAuto)
	{  /*we just went from manual to auto*/
		PID_Initialize();
	}
	inAuto = newAuto;
}

/* Initialize()****************************************************************
*	does all the things that need to happen to ensure a bumpless transfer
*  from manual to automatic mode.
******************************************************************************/
void PID_Initialize()
{
	ITerm = 0;
	lastInput = 0;  // might need to be fixed later, was *myInput
	if(ITerm > outMax) ITerm = outMax;
	else if(ITerm < outMin) ITerm = outMin;
}

/* SetControllerDirection(...)*************************************************
* The PID will either be connected to a DIRECT acting process (+Output leads
* to +Input) or a REVERSE acting process(+Output leads to -Input.)  we need to
* know which one, because otherwise we may increase the output when we should
* be decreasing.  This is called from the constructor.
******************************************************************************/
void PID_SetControllerDirection(int Direction)
{
	if(inAuto && Direction !=controllerDirection)
	{
		kp = (0 - kp);
		ki = (0 - ki);
		kd = (0 - kd);
	}
	controllerDirection = Direction;
}

/* Status Funcions*************************************************************
* Just because you set the Kp=-1 doesn't mean it actually happened.  these
* functions query the internal state of the PID.  they're here for display
* purposes.  this are the functions the PID Front-end uses for example
******************************************************************************/
float PID_GetKp(){ return  dispKp; }
float PID_GetKi(){ return  dispKi;}
float PID_GetKd(){ return  dispKd;}
int PID_GetMode(){ return  inAuto ? AUTOMATIC : MANUAL;}
int PID_GetDirection(){ return controllerDirection;}
