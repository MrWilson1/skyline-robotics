#ifndef TEST_ENCODER_OI_H
#define TEST_ENCODER_OI_H

#include "WPILib.h"

#include "Xbox.h"
#include "BaseOI.h"
#include "../Commands/DriveCommands.h"
#include "../Commands/TestCommands.h"
#include "../Subsystems/Drive/DriveSubsystem.h"
#include "../Subsystems/Experimental.h"

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
	TestEncoder *m_leftEncoderTest;
	TestEncoder *m_rightEncoderTest;
	
	Command *m_teleopCommand;
	Command *m_testLeftEncoderCommand;
	Command *m_testRightEncoderCommand;
	
	JoystickButton *m_driveStraightButton;
};

#endif
