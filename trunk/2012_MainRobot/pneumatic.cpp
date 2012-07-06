#include "pneumatic.h"

Pneumatic::Pneumatic() {
	mCompressor = new Compressor(0,0,0,0);
	mSolenoid = new Solenoid(0,0);
	
	mCompressor->Start();
	mSolenoid->Set(false);
}

Pneumatic::~Pneumatic()
{
	//empty
}

void Pneumatic::Open()
{
	mSolenoid->Set(false);
}

void Pneumatic::Close()
{
	mSolenoid->Set(true);
}

PneumaticController::PneumaticController(Pneumatic *pneumatic, Joystick *joystick) {
	mPneumatic = pneumatic;
	mJoystick = joystick;
}

void PneumaticController::Run(void) {
	bool open = mJoystick->GetRawButton(1);
	bool close = mJoystick->GetRawButton(2);
	
	if (open) {
		mPneumatic->Open();
	} else if (close) {
		mPneumatic->Close();
	} else {
		// do nothing
	}
}
