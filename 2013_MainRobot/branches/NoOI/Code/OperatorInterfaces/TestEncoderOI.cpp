#include "TestEncoderOI.h"

TestEncoderOI::TestEncoderOI(XboxController *xbox, BaseDrive *drive,
		TestEncoder *leftEncoderTest, TestEncoder *rightEncoderTest) :
		BaseOI("TestEncoderOI") {
	m_xbox = xbox;
	m_drive = drive;
	m_leftEncoderTest = leftEncoderTest;
	m_rightEncoderTest = rightEncoderTest;
	
	//m_teleopCommand = new TankDriveCommand(m_drive, this);
	
	m_testLeftEncoderCommand = new TestEncoderCommand(m_leftEncoderTest, "left Encoder");
	m_testRightEncoderCommand = new TestEncoderCommand(m_rightEncoderTest, "right Encoder");
	
	m_driveStraightButton = new JoystickButton(m_xbox, m_xbox->X);
}

TestEncoderOI::~TestEncoderOI() {
	// empty
}

void TestEncoderOI::SetupTeleop() {
	SmartDashboard::PutString("TestOI status", "Setting up teleop");
	m_drive->SetDefaultCommand(m_teleopCommand);
	m_leftEncoderTest->SetDefaultCommand(m_testLeftEncoderCommand);
	m_rightEncoderTest->SetDefaultCommand(m_testRightEncoderCommand);
	//m_driveStraightButton->WhileHeld(new TravelStraightManualCommand(m_drive, this, Left));
	

	// Various config stuff.
	std::string key = m_leftEncoderTest->GetName() + std::string(" ") + std::string("SetDistancePerPulse");
	SmartDashboard::PutNumber(key, m_leftEncoderTest->GetDistancePerPulse());

	key = m_rightEncoderTest->GetName() + std::string(" ") + std::string("SetDistancePerPulse");
	SmartDashboard::PutNumber(key, m_rightEncoderTest->GetDistancePerPulse());
	SmartDashboard::PutNumber("Left Rate P", 0.1);
	SmartDashboard::PutNumber("Left Rate I", 0.2);
	SmartDashboard::PutNumber("Left Rate D", 0);
	SmartDashboard::PutNumber("Right Rate P", 0.1);
	SmartDashboard::PutNumber("Right Rate I", 0.2);
	SmartDashboard::PutNumber("Right Rate D", 0);
	
	// SmartDashboard buttons
	SmartDashboard::PutData("Refresh PID Rate", new RefreshPidCommand(dynamic_cast<IPidDrive *>(m_drive)));
	
	SmartDashboard::PutData("left Encoder Start", new StartTestEncoderCommand(m_leftEncoderTest, "left Encoder Start"));
	SmartDashboard::PutData("right Encoder Start", new StartTestEncoderCommand(m_rightEncoderTest, "right Encoder Start"));
	
	SmartDashboard::PutData("left Encoder Stop", new StopTestEncoderCommand(m_leftEncoderTest, "left Encoder Stop"));
	SmartDashboard::PutData("right Encoder Stop", new StopTestEncoderCommand(m_rightEncoderTest, "right Encoder Stop"));
	
	SmartDashboard::PutData("left Encoder Reset", new ResetTestEncoderCommand(m_leftEncoderTest, "left Encoder Reset"));
	SmartDashboard::PutData("right Encoder Reset", new ResetTestEncoderCommand(m_rightEncoderTest, "right Encoder Reset"));

	SmartDashboard::PutData("left Encoder Update", new UpdateTestEncoderCommand(m_leftEncoderTest, "left Encoder Update"));
	SmartDashboard::PutData("right Encoder Update", new UpdateTestEncoderCommand(m_rightEncoderTest, "right Encoder Update"));
}

TankValues TestEncoderOI::GetTankValues() {
	float left = GetAxis(m_xbox->LeftY);
	float right = GetAxis(m_xbox->RightY);
	TankValues t;
	t.Left = left;
	t.Right = right;
	if (Tools::IsWithinRange(left, 0, 0.2)) {
		t.Left = 0;
	}
	if (Tools::IsWithinRange(right, 0, 0.2)) {
		t.Right = 0;
	}
	return t;
}

ArcadeValues TestEncoderOI::GetArcadeValues() {
	float move = GetAxis(m_xbox->LeftY);
	float rotate = GetAxis(m_xbox->RightX);
	ArcadeValues a;
	a.Move = move;
	a.Rotate = rotate;
	return a;
}

float TestEncoderOI::GetAxis(int axis) {
	float value = m_xbox->GetRawAxis(axis);
	// Apply filters to value here if necessary
	return value;
}

float TestEncoderOI::GetAxis(int axis, int joystickNumber) {
	float value = m_xbox->GetRawAxis(axis);
	// Apply filters to value here if necessary
	return value;
}
