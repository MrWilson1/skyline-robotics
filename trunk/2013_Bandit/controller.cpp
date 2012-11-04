#include "controller.h"

JoystickController::JoystickController(Joystick *lStick, Joystick *rStick, Joystick *tStick, map<string, Component *> *components) {
	mLeftJoystick = lStick;
	mRightJoystick = rStick;
	mTwistJoystick = tStick;
	
	mComponents = components;
	mDrive = (Drive *) mComponents->find("Drive")->second;
}

void JoystickController::Run() {
	RunDrive(mDrive);
}

void JoystickController::RunDrive(Drive *mDrive) {
	// tank drive
	mDrive->GetDrive()->TankDrive(mLeftJoystick, mRightJoystick);
}
