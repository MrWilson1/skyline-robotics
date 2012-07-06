#ifndef PNEUMATIC_H
#define PENUMATIC_H

#include "WPILib.h"

class Pneumatic
{
public:
	Pneumatic();
	~Pneumatic();
	
	void Open();
	void Close();
private:
	Compressor* mCompressor;
	Solenoid* mSolenoid;
};

class PneumaticController
{
public:
	PneumaticController(Pneumatic *pneumatic, Joystick joystick);
	void Run(void);
private:
	Pneumatic* mPneumatic;
	Joystick* mJoystick;	
};
