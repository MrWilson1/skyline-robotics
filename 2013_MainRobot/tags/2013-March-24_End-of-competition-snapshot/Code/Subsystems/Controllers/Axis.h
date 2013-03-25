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
	Axis(Joystick *joystick, const int axisNum, bool reversed=false);
	~Axis();
	float Get();
	Joystick *GetJoystick();
	const int GetAxisNum();
	
private:
	Joystick *m_joystick;
	const int m_axisNum;
	bool m_isReversed;
};


class TruncatedCurvedAxis : public BaseAxis {
public:
	TruncatedCurvedAxis(Joystick *joystick, const int axisNum, bool reversed=false);
	~TruncatedCurvedAxis();
	float Get();
	Joystick *GetJoystick();
	const int GetAxisNum();
	
private:
	Joystick *m_joystick;
	const int m_axisNum;
	bool m_isReversed;
};


class CustomRangeAxis : public BaseAxis {
public:
	CustomRangeAxis(Joystick *joystick, const int axisNum, const float minRange, const float maxRange);
	~CustomRangeAxis();
	float Get();
	Joystick *GetJoystick();
	const int GetAxisNum();
	
private:
	Joystick *m_joystick;
	const int m_axisNum;
	const float m_minRange;
	const float m_maxRange;
};


class ButtonAxis : public BaseAxis {
public:
	ButtonAxis(Button *buttonUp, Button *buttonDown);
	~ButtonAxis();
	float Get();
	Joystick *GetJoystick();
	const int GetAxisNum();
	
private:
	Button *m_buttonUp;
	Button *m_buttonDown;
};


#endif
