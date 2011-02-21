
#include "LiftController.h"


LiftController::LiftController (
		UINT32 motorPort,
		UINT32 highLimitPort,
		UINT32 lowLimitPort)
{
	m_liftMotor = new Victor(motorPort);
	m_highLimit = new DigitalInput (highLimitPort);
	m_lowLimit = new DigitalInput(lowLimitPort);
}



LiftController::~LiftController ()
{
	if (m_liftMotor)
		delete m_liftMotor;
	
	if (m_highLimit)
		delete m_highLimit;
	
	if (m_lowLimit)
		delete m_lowLimit;
}



bool
LiftController::isAtTop()
{
	bool result;

	if (m_highLimit->Get())
		result = true;
	else
		result = false;
	
	return (result);
}



bool
LiftController::isAtBottom()
{
	bool result;

	if (m_lowLimit->Get())
		result = true;
	else
		result = false;
	
	return (result);
}



bool
LiftController::stop()
{
	m_liftMotor->Set(0);
	
	return true;
}



/****************************************************************************
 * Extends the lift
 * 
 * Input -
 * 		The speed at which to extend the lift
 * Output - 
 * 		Returns true if the lift is moving up
 * 		Resurns false if the lift is already at the top or the speed is out
 * 		of range.
 ***************************************************************************/
bool
LiftController::extend(float speed)
{
	bool result;
	
	if (isAtTop())
	{
		m_liftMotor->Set(0);
		result = false;
	}
	else if (speed >= 0 && speed <= 1.0)
	{
		m_liftMotor->Set(speed); 
		result = true;
	}
	else
		result = false;
	
	return result;
}



/****************************************************************************
 * Retracts the lift
 * 
 * Input -
 * 		The speed at which to retract the lift
 * Output - 
 * 		Returns true if the lift is moving down
 * 		Resurns false if the lift is already at the bottom or the speed is
 * 		out of range.
 ***************************************************************************/
bool
LiftController::retract(float speed)
{
	bool result;
	
	if (isAtBottom())
	{
		m_liftMotor->Set(0);
		result = false;
	}
	else if (speed >= -1.0 && speed <= 0)
	{
		m_liftMotor->Set(speed); 
		result = true;
	}
	else
		result = false;
	
	return result;
}
