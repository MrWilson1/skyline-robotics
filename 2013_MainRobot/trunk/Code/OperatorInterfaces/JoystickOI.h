#ifndef JOYSTICK_OI_H
#define JOYSTICK_OI_H

#include "WPILib.h"
#include "BaseOI.h"
#include "..\Commands\DriveCommands.h"
#include "..\Subsystems\Drive\DriveSubsystem.h"

class JoystickOI : public BaseOI {
public:
	JoystickOI(Joystick *rightJoystick, Joystick *leftJoystick, Joystick *twistJoystick, BaseDrive *drive);
	~JoystickOI();
	
	void SetupTeleop();
	TankValues GetTankValues();
	ArcadeValues GetArcadeValues();
	float GetAxis(int axis);
	float GetAxis(int axis, int joystickNumber);
	
private:
	Joystick *m_leftJoystick; // joystick 1
	Joystick *m_rightJoystick; // joystick 2
	Joystick *m_twistJoystick; // joystick 3
	BaseDrive *m_drive;
	
	Command *m_teleopCommand;
};

#endif

