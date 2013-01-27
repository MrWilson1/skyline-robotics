#include "TestEncoderOI.h"

TestEncoderOI::TestEncoderOI(XboxController *xbox, BaseDrive *drive,
		TestEncoder *leftEncoderTest, TestEncoder *rightEncoderTest,
		TestMotor *testMotor) :
		BaseOI("TestEncoderOI") {
	m_xbox = xbox;
	m_drive = drive;
	m_leftEncoderTest = leftEncoderTest;
	m_rightEncoderTest = rightEncoderTest;
	m_testMotor = testMotor;
	
	m_teleopCommand = new TankDriveCommand(m_drive, this);
	m_testLeftEncoderCommand = new TestEncoderCommand(m_leftEncoderTest);
	m_testRightEncoderCommand = new TestEncoderCommand(m_rightEncoderTest);
	m_testMotorCommand1 = new TestMotorCommand(testMotor, 0.5);
	m_testMotorCommand2 = new TestMotorCommand(testMotor, -0.5);
	
	m_command1Button = new JoystickButton(m_xbox, m_xbox->A);
	m_command2Button = new JoystickButton(m_xbox, m_xbox->B);
}

TestEncoderOI::~TestEncoderOI() {
	// empty
}

void TestEncoderOI::SetupTeleop() {
	SmartDashboard::PutString("TestOI status", "Setting up teleop");
	m_teleopCommand->Start();
	m_testLeftEncoderCommand->Start();
	m_testRightEncoderCommand->Start();
	
	m_command1Button->WhenPressed(m_testMotorCommand1);
	m_command2Button->WhenPressed(m_testMotorCommand2);
	
}

TankValues TestEncoderOI::GetTankValues() {
	float left = GetAxis(m_xbox->LeftY);
	float right = GetAxis(m_xbox->RightY);
	TankValues t;
	t.Left = left;
	t.Right = right;
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
