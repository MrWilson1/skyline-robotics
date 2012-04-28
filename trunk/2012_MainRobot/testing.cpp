#include "testing.h"

ServoController::ServoController(Servo *servo, Joystick *joystick) :
	BaseController()
{
	mServo = servo;
	mJoystick = joystick;
}

void ServoController::Run()
{
	if (mJoystick->GetTrigger()) {
		float amount = Tools::Coerce(
				mJoystick->GetY(),
				-1.0,
				1.0,
				0.0,
				1.0);
		SmartDashboard::GetInstance()->Log(amount, "(SERVO) amount:");
		mServo->Set(amount);
	}
	SmartDashboard::GetInstance()->Log(mServo->Get(), "(SERVO) get:");
}
