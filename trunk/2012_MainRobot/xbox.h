#ifndef _XBOX_CONTROLLER_H
#define _XBOX_CONTROLLER_H

#include "WPILib.h"
#include "components.h"

class XboxController : public Joystick
{
public:
	enum Axis {
		LeftX,
		LeftY,
		Bumper,
		RightY,
		RightX
	};
	
	enum Button {
		A,
		X,
		B,
		Y,
		LeftBumper,
		RightBumper,
		Back,
		Start,
		LeftClick,
		RightClick
	};
	
	static const UINT32 numAxisTypes = 5;
	static const UINT32 numButtonTypes = 10;
	
	XboxController(UINT32);
	
	float GetAxis(Axis);
	bool GetButton(Button);
};

class XboxTest : public BaseController
{
public:
	XboxTest(XboxController *);
	void Run();
private:
	XboxController *mXboxController;
};

#endif
