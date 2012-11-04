#ifndef CONTROLLER_H_
#define CONTROLLER_H_

#include "WPILib.h"
#include "component.h"
#include "drive.h"

class JoystickController : public Component {
public:
	Joystick *mLeftJoystick;
	Joystick *mRightJoystick;
	Joystick *mTwistJoystick;
	map<string, Component *> *mComponents;
	Drive *mDrive;
	void Run(void);
	JoystickController(Joystick *, Joystick *, Joystick *, map<string, Component *> *);
protected:
	void RunDrive(Drive *);
};

#endif
