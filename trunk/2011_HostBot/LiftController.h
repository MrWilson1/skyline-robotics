#ifndef LIFT_CONTROLLER_H
#define LIFT_CONTROLLER_H

#include "WPILib.h"

class LiftController
{
	private:
		SpeedController * m_liftMotor;	// Controls the lift
		DigitalInput * m_highLimit;		// The high limit for the lift
		DigitalInput * m_lowLimit;		// The lower limit for the lift
		
	public:


	public:
		LiftController (
				UINT32 motorPort,
				UINT32 highLimitPort,
				UINT32 lowLimitPort);
		~LiftController ();

		bool isAtTop();
		bool isAtBottom();

		bool stop();
		bool extend(float speed);
		bool retract(float speed);
		

};


#endif	// LIFT_CONTROLLER_H
