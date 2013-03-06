#include "DriveSubsystem.h"

BaseDrive::BaseDrive(const char *name) :
	BaseSubsystem(name) {
	// empty
}

BaseDrive::~BaseDrive() {
	// empty
}

IPidDrive::IPidDrive() {
	// empty
}

IPidDrive::~IPidDrive() {
	// empty
}


SimpleDrive::SimpleDrive(
		Tread *left, 
		Tread *right) :
		BaseDrive("SimpleDrive") {
	m_leftTread = left;
	m_rightTread = right;
	m_rightTread->SetDirection(Tread::kReverse);
	AddActuatorToLiveWindow("Left", m_leftTread);
	AddActuatorToLiveWindow("Right", m_rightTread);
}

SimpleDrive::~SimpleDrive() {
	// empty
}

void SimpleDrive::TankDrive(float leftValue, float rightValue) {
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

void SimpleDrive::TankDrive(float leftValue, float rightValue, bool squaredInputs) {
	if (squaredInputs) {
		leftValue = Tools::SquareMagnitude(leftValue);
		rightValue = Tools::SquareMagnitude(rightValue);
	}
	TankDrive(leftValue, rightValue);
}

void SimpleDrive::ResetDistanceAndRotation() {
	// do nothing
}

void SimpleDrive::TravelDistance(float distanceInInches) {
	// do nothing
}

void SimpleDrive::Rotate(float degrees) {
	// do nothing
}

void SimpleDrive::Disable() {
	Brake();
}

void SimpleDrive::Enable() {
	// empty
}

/**
 * Tells the robot to stop moving, accounting for things like
 * slipping and sliding.
 * 
 * On the SimpleDrive, this is equivalent to calling 
 * SimpleDrive::StopMoving.
 */
void SimpleDrive::Brake() {
	TankDrive(0.0, 0.0);
}



SmoothEncoder::SmoothEncoder(
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

SmoothEncoder::~SmoothEncoder() {
	// Empty
}

double SmoothEncoder::PIDGet() {
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

Encoder * SmoothEncoder::GetEncoder() {
	return m_encoder;
}

void SmoothEncoder::SetMaxSize(unsigned int maxSize) {
	m_maxSize = maxSize;
	while (m_history.size() > m_maxSize) {
		m_history.pop_back();
	}
}

unsigned int SmoothEncoder::GetMaxSize() {
	return m_maxSize;
}
	
void SmoothEncoder::SetPIDSourceParameter(Encoder::PIDSourceParameter pidSourceParameter) {
	m_pidSourceParameter = pidSourceParameter;
	m_encoder->SetPIDSourceParameter(m_pidSourceParameter);
}

Encoder::PIDSourceParameter SmoothEncoder::GetPIDSourceParameter() {
	return m_pidSourceParameter;
}

void SmoothEncoder::SetDistancePerPulse(float distancePerPulse) {
	m_distancePerPulse = distancePerPulse;
	m_encoder->SetDistancePerPulse(m_distancePerPulse);
}

float SmoothEncoder::GetDistancePerPulse() {
	return m_distancePerPulse;
}

void SmoothEncoder::Reset() {
	m_encoder->Reset();
}



IntegratedPid::IntegratedPid(float p, float i, float d, float dpp, 
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

IntegratedPid::~IntegratedPid() {
	// empty
}
	
void IntegratedPid::SetRaw(float speed) {
	m_tread->SetSpeed(speed);
}

void IntegratedPid::SetSetpoint(float setpoint) {
	//setpoint *= m_dpp;
	m_pid->SetSetpoint(setpoint);
}

void IntegratedPid::SetPid(float p, float i, float d) {
	m_p = p;
	m_i = i;
	m_d = d;
	m_pid->SetPID(p, i, d);
}

void IntegratedPid::SetDistancePerPulse(float dpp) {
	m_dpp = dpp;
	m_encoder->SetDistancePerPulse(1.0f/dpp);
}

float IntegratedPid::GetSpeed() {
	return m_tread->GetSpeed();
}

void IntegratedPid::Reset() {
	m_encoder->Reset();
}

void IntegratedPid::Enable() {
	m_encoder->SetDistancePerPulse(1.0f/m_dpp);
	m_encoder->SetPIDSourceParameter(m_pidSource);
	m_encoder->Start();
	m_pid->Enable();
}

void IntegratedPid::Disable() {
	m_encoder->Reset();
	//m_encoder->Stop();
	m_pid->Disable();
}

PIDController* IntegratedPid::GetController() {
	return m_pid;
}

float IntegratedPid::GetP() {
	return m_p;
}
	
float IntegratedPid::GetI() {
	return m_i;
}

float IntegratedPid::GetD() {
	return m_d;
}

float IntegratedPid::GetDistancePerPulse() {
	return m_dpp;
}

float IntegratedPid::GetDistance() {
	return m_encoder->GetDistance();
}


DrivePid::DrivePid(IntegratedPid leftPid, IntegratedPid rightPid) :
		Left(leftPid),
		Right(rightPid) {
	// empty
}

DrivePid::~DrivePid() {
	// empty
}

void DrivePid::SetSetpoint(float left, float right) {
	Left.SetSetpoint(left);
	Right.SetSetpoint(right);
}

void DrivePid::Reset() {
	Left.Reset();
	Right.Reset();
}

void DrivePid::Enable() {
	Left.Enable();
	Right.Enable();
}

void DrivePid::Disable() {
	Left.Disable();
	Right.Disable();
}




PidSimpleDrive::PidSimpleDrive(DrivePid ratePid, DrivePid distancePid, double robotDiagonalLength) :
			BaseDrive("PidSimpleDrive"),
			IPidDrive(),
			m_currentMode(Rate),
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

PidSimpleDrive::~PidSimpleDrive() {
	// empty
}

void PidSimpleDrive::TankDrive(float leftValue, float rightValue) {
	TryToggling(Rate);
	TankDrive(leftValue, rightValue, false);
}

void PidSimpleDrive::TankDrive(float leftValue, float rightValue, bool squaredInputs) {
	TryToggling(Rate);
	if (squaredInputs) {
		leftValue = Tools::SquareMagnitude(leftValue);
		rightValue = Tools::SquareMagnitude(rightValue);
	}
	m_ratePid.SetSetpoint(leftValue, rightValue);
	SmartDashboard::PutNumber("Left Tread", m_ratePid.Left.GetSpeed());
	SmartDashboard::PutNumber("Right Tread", m_ratePid.Right.GetSpeed());
}

void PidSimpleDrive::ResetDistanceAndRotation() {
	m_ratePid.Reset();
	m_distancePid.Reset();
}

void PidSimpleDrive::TravelDistance(float distanceInInches) {
	TryToggling(Distance);
	m_distancePid.SetSetpoint(distanceInInches, distanceInInches);
}

void PidSimpleDrive::Rotate(float degrees) {
	TryToggling(Distance);
	double radius = m_robotDiagonalLength / 2;
	double circumference = Tools::PI * radius * radius;
	double arcLength = circumference * (degrees / 360);
	m_distancePid.SetSetpoint(arcLength, -arcLength);
}

void PidSimpleDrive::Disable() {
	m_ratePid.Left.SetRaw(0);
	m_ratePid.Right.SetRaw(0);
	m_ratePid.Disable();
	m_distancePid.Disable();
}

void PidSimpleDrive::Enable() {
	m_ratePid.Enable();
}

void PidSimpleDrive::Brake() {
	TankDrive(0.0, 0.0);
}

void PidSimpleDrive::TryToggling(PidMode mode) {
	if (mode == m_currentMode) {
		return;
	}
	switch (mode) {
	case Rate:
		m_distancePid.Disable();
		m_ratePid.Enable();
		m_currentMode = Rate;
		break;
	case Distance:
		m_ratePid.Disable();
		m_distancePid.Enable();
		m_currentMode = Distance;
		break;
	default:
		SmartDashboard::PutString("PidSimpleDrive Toggling error", "Unknown option");
	}
}

void PidSimpleDrive::UpdatePidValues() {
	// currently not implemented.
}

void PidSimpleDrive::AdjustRatePid(float lp, float li, float ld, float rp, float ri, float rd) {
	m_ratePid.Left.SetPid(lp, li, ld);
	m_ratePid.Right.SetPid(rp, ri, rd);
}
	
void PidSimpleDrive::AdjustDistancePid(float lp, float li, float ld, float rp, float ri, float rd) {
	m_distancePid.Left.SetPid(lp, li, ld);
	m_distancePid.Right.SetPid(rp, ri, rd);
}

DrivePid PidSimpleDrive::GetRatePid() {
	return m_ratePid;
}

DrivePid PidSimpleDrive::GetDistancePid() {
	return m_distancePid;
}

void PidSimpleDrive::SetRatePid(DrivePid ratePid) {
	if (m_currentMode == Rate) {
		m_ratePid.Disable();
	}
	m_ratePid = ratePid;
	m_ratePid.Enable();
}

void PidSimpleDrive::SetDistancePid(DrivePid distancePid) {
	if (m_currentMode == Distance) {
		m_distancePid.Disable();
	}
	m_distancePid = distancePid;
	m_distancePid.Disable();
}
