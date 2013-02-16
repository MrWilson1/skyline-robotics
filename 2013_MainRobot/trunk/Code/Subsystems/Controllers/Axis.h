#ifndef AXIS_H
#define AXIS_H

#include "WPILib.h"

class Axis {
public:
	Axis(Joystick *joystick, int axisNum);
	~Axis();
	float Get();
	Joystick *GetJoystick();
	const int GetAxisNum();
	
private:
	Joystick *m_joystick;
	const int m_axisNum;
};




#endif
