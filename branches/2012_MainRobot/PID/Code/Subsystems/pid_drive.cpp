#include "pid_drive.h"

EncoderSource::EncoderSource(Encoder *encoder) :
		PIDSource() 
{
	mEncoder = encoder;
	mEncoder->SetPIDSourceParameter(mEncoder->kRate);
	mEncoder->Start();
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
	mLeftPid = new PIDController(0, 0, 0, mLeftEncoderSource, mLeftTread);
	mRightPid = new PIDController(0, 0, 0, mRightEncoderSource, mRightTread);
	
	mLeftPid->SetOutputRange(-1.0, 1.0);
	mRightPid->SetOutputRange(-1.0, 1.0);
	
	mState = kManual;
	
	SmartDashboard *s = SmartDashboard::GetInstance();
	s->PutString("Left P", "1.0");
	s->PutString("Left I", "0.0");
	s->PutString("Left D", "0.0");
	
	s->PutString("Right P", "1.0");
	s->PutString("Right I", "0.0");
	s->PutString("Right D", "0.0");
	
	s->PutString("PID Tune", "disable");
	s->PutString("PID Enabled", "enable");
	mLeftPid->Enable();
	mRightPid->Enable();
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
	// temporary
	left = -left;
	
	SmartDashboard *s = SmartDashboard::GetInstance();
	float left_p = Tools::StringToFloat(s->GetString("Left P"));
	float left_i = Tools::StringToFloat(s->GetString("Left I"));
	float left_d = Tools::StringToFloat(s->GetString("Left D"));
	
	float right_p = Tools::StringToFloat(s->GetString("Right P"));
	float right_i = Tools::StringToFloat(s->GetString("Right I"));
	float right_d = Tools::StringToFloat(s->GetString("Right D"));
	
	string tune = s->GetString("PID Tune");
	if (tune != "disable") {
		mLeftPid->Disable();
		mRightPid->Disable();
		mLeftPid->SetPID(left_p, left_i, left_d);
		mRightPid->SetPID(right_p, right_i, right_d);
		mLeftPid->Enable();
		mRightPid->Enable();
		s->PutString("PID Tune", "disable");
	}

	string pidState = s->GetString("PID Enabled");
	if (pidState == "enable") {
		if (mState == kManual) {
			s->Log("Manual", "PID Drive state");
			mLeftPid->SetSetpoint(left);
			mRightPid->SetSetpoint(right);
		} else if (mState == kStraight) {
			s->Log("Straight", "PID Drive state");
			float speed = (left + right) / 2;
			mLeftPid->SetSetpoint(speed);
			mRightPid->SetSetpoint(speed);
		} else if (mState == kHalt) {
			s->Log("Halt", "PID Drive state");
			mLeftPid->SetSetpoint(0);
			mRightPid->SetSetpoint(0);
		}
	} else {
		mLeftPid->Disable();
		mRightPid->Disable();
		if (mState == kManual) {
			s->Log("Manual no PID", "PID Drive state");
			mLeftTread->PIDWrite(left);
			mRightTread->PIDWrite(right);
		} else if (mState == kStraight) {
			s->Log("Straight no PID", "PID Drive state");
			float speed = (left + right) / 2;
			mLeftTread->PIDWrite(speed);
			mRightTread->PIDWrite(speed);
		} else if (mState == kHalt) {
			s->Log("Halt", "PID Drive state");
			mLeftTread->PIDWrite(0);
			mRightTread->PIDWrite(0);
		}
		mLeftPid->Enable();
		mRightPid->Enable();		
	}
	
	
	s->Log(left, "Input left");
	s->Log(right, "Input right");
	s->Log(mLeftPid->Get(), "Output left");
	s->Log(mRightPid->Get(), "Output right");
	s->Log(mLeftEncoderSource->mEncoder->GetRate(), "Left Encoder Rate");
	s->Log(mRightEncoderSource->mEncoder->GetRate(), "Right Encoder Rate");
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
