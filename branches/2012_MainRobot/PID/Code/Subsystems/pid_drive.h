#ifndef PID_DRIVE_H_
#define PID_DRIVE_H_

#include <string>

#include "WPIlib.h"
#include "../Definitions/components.h"
#include "../tools.h"

class EncoderSource : public PIDSource {
public:
	EncoderSource(Encoder *);
	double PIDGet();
	Encoder *mEncoder;
	// The encoder is publically exposed for now to help 
	// aid in configuration and debugging
};

class Tread : public PIDOutput {
public:
	Tread(SpeedController *frontWheel, SpeedController *backWheel);
	void PIDWrite(float);
protected:
	SpeedController *mFrontWheel;
	SpeedController *mBackWheel;
};

class PidDrive : public BaseComponent {
public:
	PidDrive(SpeedController *leftFrontDrive,
			 SpeedController *leftBackDrive,
			 SpeedController *rightFrontDrive,
			 SpeedController *rightBackDrive,
			 Encoder *leftEncoder,
			 Encoder *rightEncoder);
	~PidDrive();
	enum State {
		kStraight,
		kManual,
		kHalt
	};
	void SetState(State);
	State GetState();
	void TankDrive(float left, float right);
protected:
	EncoderSource *mLeftEncoderSource;
	EncoderSource *mRightEncoderSource;
	Tread *mLeftTread;
	Tread *mRightTread;
	SendablePIDController *mLeftPid;
	SendablePIDController *mRightPid;
	State mState;
};

class PidDriveController : public BaseController {
public:
	PidDriveController(PidDrive *, Joystick *left, Joystick *right);
	~PidDriveController();
	void Run();
protected:
	PidDrive *mPidDrive;
	Joystick *mLeftStick;
	Joystick *mRightStick;
	string mPreviousCommand;
};

#endif
