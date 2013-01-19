#ifndef XBOX_OI_H
#define XBOX_OI_H

#include "WPILib.h"

#include "Xbox.h"
#include "BaseOI.h"
#include "../Commands/DriveCommands.h"
#include "../Commands/TestCommands.h"
#include "../Subsystems/DriveSubsystem.h"
#include "../Subsystems/Experimental.h"

/**
 * A simple operator interface using the Xbox Controllers
 */
class XboxOI : public BaseOI
{
public:
	XboxOI(XboxController *xbox, BaseDrive *drive);
	~XboxOI();
	
	void SetupTeleop();
	TankValues GetTankValues();
	ArcadeValues GetArcadeValues();
	float GetAxis(int axis);
	
private:
	XboxController *m_xbox;
	BaseDrive *m_drive;
	
	JoystickButton *m_spasmButton;
	Command *m_teleopCommand;
};

class TestEncoderOI : public BaseOI
{
public:
	TestEncoderOI(XboxController *xbox, BaseDrive *drive, 
			TestEncoder *leftEncoderTest, TestEncoder *rightEncoderTest);
	~TestEncoderOI();
	
	void SetupTeleop();
	TankValues GetTankValues();
	ArcadeValues GetArcadeValues();
	float GetAxis(int axis);
	
private:
	XboxController *m_xbox;
	BaseDrive *m_drive;
	
	Command *m_teleopCommand;
	Command *m_testLeftEncoderCommand;
	Command *m_testRightEncoderCommand;
	TestEncoder *m_leftEncoderTest;
	TestEncoder *m_rightEncoderTest;
};

#endif
