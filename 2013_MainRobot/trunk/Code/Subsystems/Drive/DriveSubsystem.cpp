#include "DriveSubsystem.h"

Drive::Base::Base(const char *name) :
	BaseSubsystem(name) {
	// empty
}

Drive::Base::~Base() {
	// empty
}

Drive::IPid::IPid() {
	// empty
}

Drive::IPid::~IPid() {
	// empty
}


Drive::Simple::Simple(
		Tread *left, 
		Tread *right) :
		Drive::Base("SimpleDrive") {
	m_leftTread = left;
	m_rightTread = right;
	m_rightTread->SetDirection(Tread::kReverse);
	AddActuatorToLiveWindow("Left", m_leftTread);
	AddActuatorToLiveWindow("Right", m_rightTread);
}

Drive::Simple::~Simple() {
	// empty
}

void Drive::Simple::TankDrive(float leftValue, float rightValue) {
	if (m_leftTread == NULL) {
		SmartDashboard::PutString(GetName(), "leftTread null");
		return;
	}
	if (m_rightTread == NULL) {
		SmartDashboard::PutString(GetName(), "rightTread null");
		return;
	}
	SmartDashboard::PutString(GetName(), "safe?");
	m_leftTread->SetSpeed(leftValue);
	m_rightTread->SetSpeed(rightValue);
}

void Drive::Simple::TankDrive(float leftValue, float rightValue, bool squaredInputs) {
	if (squaredInputs) {
		leftValue = Tools::SquareMagnitude(leftValue);
		rightValue = Tools::SquareMagnitude(rightValue);
	}
	TankDrive(leftValue, rightValue);
}

void Drive::Simple::ResetDistanceAndRotation() {
	// do nothing
}

void Drive::Simple::TravelDistance(float distanceInInches) {
	// do nothing
}

void Drive::Simple::Rotate(float degrees) {
	// do nothing
}

void Drive::Simple::Disable() {
	Brake();
}

void Drive::Simple::Enable() {
	// empty
}

/**
 * Tells the robot to stop moving, accounting for things like
 * slipping and sliding.
 * 
 * On the SimpleDrive, this is equivalent to calling 
 * Drive::Simple::StopMoving.
 */
void Drive::Simple::Brake() {
	TankDrive(0.0, 0.0);
}



Drive::SmoothEncoder::SmoothEncoder(
	Encoder *encoder, 
	unsigned int maxSize,
	Encoder::PIDSourceParameter pidSourceParameter,
	float distancePerPulse) :
		PIDSource(),
		m_maxSize(maxSize),
		m_pidSourceParameter(pidSourceParameter),
		m_distancePerPulse(distancePerPulse) {
	m_encoder = encoder;
	m_encoder->SetPIDSourceParameter(m_pidSourceParameter);
	m_encoder->SetDistancePerPulse(m_distancePerPulse);
}

Drive::SmoothEncoder::~SmoothEncoder() {
	// Empty
}

double Drive::SmoothEncoder::PIDGet() {
	double newValue = m_encoder->PIDGet();
	m_history.push_front(newValue);
	if (m_history.size() > m_maxSize) {
		m_history.pop_back();
	}
	double sum = 0;
	for (unsigned int i=0; i < m_history.size(); i++) {
		sum += m_history.at(i);
	}
	double average = sum / m_history.size();
	return average;
}

Encoder * Drive::SmoothEncoder::GetEncoder() {
	return m_encoder;
}

void Drive::SmoothEncoder::SetMaxSize(unsigned int maxSize) {
	m_maxSize = maxSize;
	while (m_history.size() > m_maxSize) {
		m_history.pop_back();
	}
}

unsigned int Drive::SmoothEncoder::GetMaxSize() {
	return m_maxSize;
}
	
void Drive::SmoothEncoder::SetPIDSourceParameter(Encoder::PIDSourceParameter pidSourceParameter) {
	m_pidSourceParameter = pidSourceParameter;
	m_encoder->SetPIDSourceParameter(m_pidSourceParameter);
}

Encoder::PIDSourceParameter Drive::SmoothEncoder::GetPIDSourceParameter() {
	return m_pidSourceParameter;
}

void Drive::SmoothEncoder::SetDistancePerPulse(float distancePerPulse) {
	m_distancePerPulse = distancePerPulse;
	m_encoder->SetDistancePerPulse(m_distancePerPulse);
}

float Drive::SmoothEncoder::GetDistancePerPulse() {
	return m_distancePerPulse;
}

void Drive::SmoothEncoder::Reset() {
	m_encoder->Reset();
}



Drive::TreadPid::TreadPid(float p, float i, float d, float dpp, 
		Encoder::PIDSourceParameter pidSource, Encoder *encoder, Tread *tread) :
		m_p(p),
		m_i(i),
		m_d(d),
		m_dpp(dpp),
		m_pidSource(pidSource) {
	m_encoder = encoder;
	m_tread = tread;
	m_pid = new PIDController(p, i, d, m_encoder, m_tread);
	m_pid->SetOutputRange(-1.0, 1.0);
	m_pid->Disable();
}

Drive::TreadPid::~TreadPid() {
	// empty
}
	
void Drive::TreadPid::SetRaw(float speed) {
	m_tread->SetSpeed(speed);
}

void Drive::TreadPid::SetSetpoint(float setpoint) {
	//setpoint *= m_dpp;
	m_pid->SetSetpoint(setpoint);
}

void Drive::TreadPid::SetPid(float p, float i, float d) {
	m_p = p;
	m_i = i;
	m_d = d;
	m_pid->SetPID(p, i, d);
}

void Drive::TreadPid::SetDistancePerPulse(float dpp) {
	m_dpp = dpp;
	m_encoder->SetDistancePerPulse(1.0f/dpp);
}

float Drive::TreadPid::GetSpeed() {
	return m_tread->GetSpeed();
}

void Drive::TreadPid::Reset() {
	m_encoder->Reset();
}

void Drive::TreadPid::Enable() {
	m_encoder->SetDistancePerPulse(1.0f/m_dpp);
	m_encoder->SetPIDSourceParameter(m_pidSource);
	m_encoder->Start();
	m_pid->Enable();
}

void Drive::TreadPid::Disable() {
	m_encoder->Reset();
	//m_encoder->Stop();
	m_pid->Disable();
}

PIDController* Drive::TreadPid::GetController() {
	return m_pid;
}

float Drive::TreadPid::GetP() {
	return m_p;
}
	
float Drive::TreadPid::GetI() {
	return m_i;
}

float Drive::TreadPid::GetD() {
	return m_d;
}

float Drive::TreadPid::GetDistancePerPulse() {
	return m_dpp;
}

float Drive::TreadPid::GetDistance() {
	return m_encoder->GetDistance();
}


Drive::DrivePid::DrivePid(Drive::TreadPid leftPid, Drive::TreadPid rightPid) :
		Left(leftPid),
		Right(rightPid) {
	// empty
}

Drive::DrivePid::~DrivePid() {
	// empty
}

void Drive::DrivePid::SetSetpoint(float left, float right) {
	Left.SetSetpoint(left);
	Right.SetSetpoint(right);
}

void Drive::DrivePid::Reset() {
	Left.Reset();
	Right.Reset();
}

void Drive::DrivePid::Enable() {
	Left.Enable();
	Right.Enable();
}

void Drive::DrivePid::Disable() {
	Left.Disable();
	Right.Disable();
}




Drive::SimplePid::SimplePid(DrivePid ratePid, DrivePid distancePid, double robotDiagonalLength) :
			Drive::Base("PidSimpleDrive"),
			Drive::IPid(),
			m_currentMode(kRate),
			m_ratePid(ratePid),
			m_distancePid(distancePid),
			m_robotDiagonalLength(robotDiagonalLength){
	
	m_ratePid.Enable();
	m_distancePid.Disable();
	
	AddActuatorToLiveWindow("Left PID Rate", m_ratePid.Left.GetController());
	AddActuatorToLiveWindow("Right PID Rate", m_ratePid.Right.GetController());
	AddActuatorToLiveWindow("Left PID Distance", m_distancePid.Left.GetController());
	AddActuatorToLiveWindow("Right PID Distance", m_distancePid.Right.GetController());
}

Drive::SimplePid::~SimplePid() {
	// empty
}

void Drive::SimplePid::TankDrive(float leftValue, float rightValue) {
	TryToggling(kRate);
	TankDrive(leftValue, rightValue, false);
}

void Drive::SimplePid::TankDrive(float leftValue, float rightValue, bool squaredInputs) {
	TryToggling(kRate);
	if (squaredInputs) {
		leftValue = Tools::SquareMagnitude(leftValue);
		rightValue = Tools::SquareMagnitude(rightValue);
	}
	m_ratePid.SetSetpoint(leftValue, rightValue);
	SmartDashboard::PutNumber("Left Tread", m_ratePid.Left.GetSpeed());
	SmartDashboard::PutNumber("Right Tread", m_ratePid.Right.GetSpeed());
}

void Drive::SimplePid::ResetDistanceAndRotation() {
	m_ratePid.Reset();
	m_distancePid.Reset();
}

void Drive::SimplePid::TravelDistance(float distanceInInches) {
	TryToggling(kDistance);
	m_distancePid.SetSetpoint(distanceInInches, distanceInInches);
}

void Drive::SimplePid::Rotate(float degrees) {
	TryToggling(kDistance);
	double radius = m_robotDiagonalLength / 2;
	double circumference = Tools::PI * radius * radius;
	double arcLength = circumference * (degrees / 360);
	m_distancePid.SetSetpoint(arcLength, -arcLength);
}

void Drive::SimplePid::Disable() {
	m_ratePid.Left.SetRaw(0);
	m_ratePid.Right.SetRaw(0);
	m_ratePid.Disable();
	m_distancePid.Disable();
}

void Drive::SimplePid::Enable() {
	m_ratePid.Enable();
}

void Drive::SimplePid::Brake() {
	TankDrive(0.0, 0.0);
}

void Drive::SimplePid::TryToggling(PidMode mode) {
	if (mode == m_currentMode) {
		return;
	}
	switch (mode) {
	case kRate:
		m_distancePid.Disable();
		m_ratePid.Enable();
		m_currentMode = kRate;
		break;
	case kDistance:
		m_ratePid.Disable();
		m_distancePid.Enable();
		m_currentMode = kDistance;
		break;
	default:
		SmartDashboard::PutString("PidSimpleDrive Toggling error", "Unknown option");
	}
}

void Drive::SimplePid::UpdatePidValues() {
	// currently not implemented.
}

void Drive::SimplePid::AdjustRatePid(float lp, float li, float ld, float rp, float ri, float rd) {
	m_ratePid.Left.SetPid(lp, li, ld);
	m_ratePid.Right.SetPid(rp, ri, rd);
}
	
void Drive::SimplePid::AdjustDistancePid(float lp, float li, float ld, float rp, float ri, float rd) {
	m_distancePid.Left.SetPid(lp, li, ld);
	m_distancePid.Right.SetPid(rp, ri, rd);
}

Drive::DrivePid Drive::SimplePid::GetRatePid() {
	return m_ratePid;
}

Drive::DrivePid Drive::SimplePid::GetDistancePid() {
	return m_distancePid;
}

void Drive::SimplePid::SetRatePid(DrivePid ratePid) {
	if (m_currentMode == kRate) {
		m_ratePid.Disable();
	}
	m_ratePid = ratePid;
	m_ratePid.Enable();
}

void Drive::SimplePid::SetDistancePid(DrivePid distancePid) {
	if (m_currentMode == kDistance) {
		m_distancePid.Disable();
	}
	m_distancePid = distancePid;
	m_distancePid.Disable();
}
