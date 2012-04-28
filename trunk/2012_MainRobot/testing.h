#ifndef _TESTING_H
#define _TESTING_H

#include "WPILib.h"
#include "components.h"

class ServoController : public BaseController
{
protected:
	Servo *mServo;
	Joystick *mJoystick;
	
public:
	ServoController(Servo *, Joystick *);
	void Run();
};

#endif
