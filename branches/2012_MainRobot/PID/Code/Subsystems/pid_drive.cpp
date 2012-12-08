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
	
	CalibrateEncoders(0.0003);
	
	// Implementing PIDController:
	// PIDController(proportional, integral, derivative, PIDSource, PIDOutput)
	mLeftPid = new PIDController(0, 0, 0, mLeftEncoderSource, mLeftTread);
	mRightPid = new PIDController(0, 0, 0, mRightEncoderSource, mRightTread);
	
	mLeftPid->SetOutputRange(-1.0, 1.0);
	mRightPid->SetOutputRange(-1.0, 1.0);
	
	mState = kManual;
	
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

void PidDrive::Enable()
{
	mLeftPid->Enable();
	mRightPid->Enable();
}

void PidDrive::Disable()
{
	mLeftPid->Disable();
	mRightPid->Disable();
}

void PidDrive::Tune(double left_p, double left_i, double left_d,
	 	            double right_p, double right_i, double right_d)
{
	Disable();
	mLeftPid->SetPID(left_p, left_i, left_d);
	mRightPid->SetPID(right_p, right_i, right_d);
	Enable();
}

void PidDrive::CalibrateEncoders(double distancePerPulse)
{
	mLeftEncoderSource->mEncoder->SetDistancePerPulse(distancePerPulse);
	mRightEncoderSource->mEncoder->SetDistancePerPulse(distancePerPulse);
}

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
	
	if (mState == kManual) {
		s->Log("Manual", "Current PID Drive state");
		mLeftPid->SetSetpoint(left);
		mRightPid->SetSetpoint(right);
	} else if (mState == kStraight) {
		s->Log("Straight", "Current PID Drive state");
		float speed = (left + right) / 2;
		mLeftPid->SetSetpoint(speed);
		mRightPid->SetSetpoint(speed);
	} else if (mState == kHalt) {
		s->Log("Halt", "Current PID Drive state");
		mLeftPid->SetSetpoint(0);
		mRightPid->SetSetpoint(0);
	} else {
		// default state, should never happen.
		mLeftPid->SetSetpoint(0);
		mRightPid->SetSetpoint(0);
	}
	
	s->Log(left, "Input left");
	s->Log(right, "Input right");
	s->Log(mLeftPid->Get(), "Output left");
	s->Log(mRightPid->Get(), "Output right");
	s->Log(mLeftEncoderSource->mEncoder->GetRate(), "Left Encoder Rate");
	s->Log(mRightEncoderSource->mEncoder->GetRate(), "Right Encoder Rate");
}



PidDriveController::PidDriveController(PidDrive *pidDrive, XboxController *xboxController) :
		BaseController()
{
	mPidDrive = pidDrive;
	mXboxController = xboxController;
	
	SmartDashboard::GetInstance()->PutString("PidDrive state", "manual");
	mPreviousCommand = "manual";
	
	SmartDashboard *s = SmartDashboard::GetInstance();
	s->PutString("Left P", "4.0");
	s->PutString("Left I", "0.0");
	s->PutString("Left D", "0.0");
	
	s->PutString("Right P", "4.0");
	s->PutString("Right I", "0.0");
	s->PutString("Right D", "0.0");
	
	s->PutString("PID Tune", "disable");
	s->PutString("PID Enabled", "enable");
	
	s->PutString("Encoder distance per pulse", "0.0003");
}

PidDriveController::~PidDriveController()
{
	// empty
}


void PidDriveController::Run()
{
	TryTuning();
	TrySetState();
	
	if (!mXboxController->GetButton(mXboxController->B)) {
		mPidDrive->TankDrive(mXboxController->GetAxis(mXboxController->LeftY), 
							 mXboxController->GetAxis(mXboxController->RightY));
	} else {
		mPidDrive->TankDrive(0, 0);
	}
}

void PidDriveController::TryTuning()
{
	SmartDashboard *s = SmartDashboard::GetInstance();
	
	double left_p = Tools::StringToFloat(s->GetString("Left P"));
	double left_i = Tools::StringToFloat(s->GetString("Left I"));
	double left_d = Tools::StringToFloat(s->GetString("Left D"));
	
	double right_p = Tools::StringToFloat(s->GetString("Right P"));
	double right_i = Tools::StringToFloat(s->GetString("Right I"));
	double right_d = Tools::StringToFloat(s->GetString("Right D"));
	
	double distancePerPulse = Tools::StringToFloat(s->GetString("Encoder distance per pulse"));
	
	if (mXboxController->GetButton(mXboxController->A)) {
		mPidDrive->Tune(left_p, left_i, left_d, right_p, right_i, right_d);
		mPidDrive->CalibrateEncoders(distancePerPulse);
	}
}

void PidDriveController::TrySetState()
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
}
