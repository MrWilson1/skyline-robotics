#ifndef AXIS_H
#define AXIS_H

#include "WPILib.h"
#include "../../Misc/Tools.h"

class BaseAxis {
public:
	BaseAxis();
	virtual ~BaseAxis();
	virtual float Get() = 0;
	virtual Joystick *GetJoystick() = 0;
	virtual const int GetAxisNum() = 0;
};

class Axis : public BaseAxis{
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


class TruncatedCurvedAxis : public BaseAxis {
public:
	TruncatedCurvedAxis(Joystick *joystick, int axisNum);
	~TruncatedCurvedAxis();
	float Get();
	Joystick *GetJoystick();
	const int GetAxisNum();
	
private:
	Joystick *m_joystick;
	const int m_axisNum;
};


#endif
