#include "Xbox.h"

XboxController::XboxController(UINT32 port) : 
		Joystick(port, numAxisTypes, numButtonTypes)
{
	// Empty
}

float XboxController::GetAxis(Axis axisName)
{
	return GetRawAxis(axisName);
}

int XboxController::GetDpad(Dpad dpad)
{
	return (int) GetRawAxis(dpad);
}

bool XboxController::GetButton(Button buttonName)
{
	return GetRawButton(buttonName);
}

