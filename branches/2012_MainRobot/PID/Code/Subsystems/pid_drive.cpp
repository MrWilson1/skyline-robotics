#include "pid_drive.h"

EncoderSource::EncoderSource(Encoder *encoder) :
		PIDSource() 
{
	mEncoder = encoder;
	mEncoder->SetPIDSourceParameter(mEncoder->kRate);
	// Set pulse distance here?
}

double EncoderSource::PIDGet() 
{
	// We could have just used the encoder directly and not create
	// a custom class, but until we have a clear idea of what to
	// do, I want to make sure we're as flexible as possible.
	return mEncoder->PIDGet();
}

Tread::Tread(SpeedController *frontWheel, SpeedController *backWheel) :
		PIDOutput()
{
	mFrontWheel = frontWheel;
	mBackWheel = backWheel;
}

void Tread::PIDWrite(float speed)
{
	speed = Tools::Limit(speed, -1.0, 1.0);
	mFrontWheel->Set(speed);
	mBackWheel->Set(speed);
}

//class PidDrive : public BaseComponent {
//public:
PidDrive::PidDrive(SpeedController *leftFrontDrive,
				   SpeedController *leftBackDrive,
				   SpeedController *rightFrontDrive,
				   SpeedController *rightBackDrive,
		           Encoder *leftEncoder,
		           Encoder *rightEncoder) :
		   BaseComponent() 
{
	mLeftTread = new Tread(leftFrontDrive, leftBackDrive);
	mRightTread = new Tread(rightFrontDrive, rightBackDrive);
	mLeftEncoderSource = new EncoderSource(leftEncoder);
	mRightEncoderSource = new EncoderSource(rightEncoder);
	
	// Implementing PIDController:
	// PIDController(proportional, integral, derivative, PIDSource, PIDOutput)
	mLeftPid = new SendablePIDController(1.0, 0, 0, mLeftEncoderSource, mLeftTread);
	mRightPid = new SendablePIDController(1.0, 0, 0, mRightEncoderSource, mRightTread);
	
	mLeftPid->SetOutputRange(-1.0, 1.0);
	mRightPid->SetOutputRange(-1.0, 1.0);
	
	mState = kManual;
}

PidDrive::~PidDrive()
{
	delete mLeftPid;
	delete mRightPid;
	delete mLeftEncoderSource;
	delete mRightEncoderSource;
	delete mLeftTread;
	delete mRightTread;
}


/*	enum State {
		kStraight,
		kManual,
		kHalt
	};*/
void PidDrive::SetState(State state)
{
	mState = state;
}

PidDrive::State PidDrive::GetState() 
{
	return mState;
}

void PidDrive::TankDrive(float left, float right)
{
	SmartDashboard::GetInstance()->PutData("Left PID:", mLeftPid);
	SmartDashboard::GetInstance()->PutData("Right PID:", mRightPid);
	
	if (mState == kManual) {
		mLeftPid->SetSetpoint(left);
		mRightPid->SetSetpoint(right);
	} else if (mState == kStraight) {
		float speed = (left + right) / 2;
		mLeftPid->SetSetpoint(speed);
		mRightPid->SetSetpoint(speed);
	} else if (mState == kHalt) {
		mLeftPid->SetSetpoint(0);
		mRightPid->SetSetpoint(0);
	}
}


PidDriveController::PidDriveController(PidDrive *pidDrive, Joystick *left, Joystick *right) :
		BaseController()
{
	mPidDrive = pidDrive;
	mLeftStick = left;
	mRightStick = right;
	SmartDashboard::GetInstance()->PutString("PidDrive state", "manual");
	mPreviousCommand = "manual";
}

PidDriveController::~PidDriveController()
{
	// empty
}

void PidDriveController::Run()
{
	SmartDashboard *s = SmartDashboard::GetInstance();
	std::string command = s->GetString("PidDrive state");
	if (command != mPreviousCommand) {
		if (command == "manual") {
			mPidDrive->SetState(mPidDrive->kManual);
		} else if (command == "straight") {
			mPidDrive->SetState(mPidDrive->kStraight);
		} else if (command == "halt") {
			mPidDrive->SetState(mPidDrive->kHalt);
		}
		mPreviousCommand = command;
	}
	mPidDrive->TankDrive(mLeftStick->GetY(), mRightStick->GetY());
}
