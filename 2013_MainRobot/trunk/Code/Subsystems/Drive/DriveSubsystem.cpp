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


SimpleDrive::SimpleDrive(RobotDrive *robotDrive) :
		BaseDrive("SimpleDrive") {
	m_robotDrive = robotDrive;
}

SimpleDrive::~SimpleDrive() {
	// empty
}

void SimpleDrive::Drive(float outputMagnitude, float curve) {
	m_robotDrive->Drive(outputMagnitude, curve);
}

void SimpleDrive::TankDrive(float leftValue, float rightValue) {
	m_robotDrive->TankDrive(leftValue, rightValue);
}

void SimpleDrive::TankDrive(float leftValue, float rightValue, bool squaredInputs) {
	m_robotDrive->TankDrive(leftValue, rightValue, squaredInputs);
}

void SimpleDrive::ArcadeDrive(float moveValue, float rotateValue) {
	m_robotDrive->ArcadeDrive(moveValue, rotateValue);
}

void SimpleDrive::ArcadeDrive(float moveValue, float rotateValue, bool squaredInputs) {
	m_robotDrive->ArcadeDrive(moveValue, rotateValue, squaredInputs);
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
	float stopValue = 0.0;
	m_robotDrive->TankDrive(stopValue, stopValue);
}



SmoothEncoder::SmoothEncoder(Encoder *encoder, unsigned int maxSize) :
		PIDSource(),
		m_maxSize(maxSize) {
	m_encoder = encoder;
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



Tread::Tread(SpeedController *front, SpeedController *back) :
	m_last(0) {
	m_front = front;
	m_back = back;
}

Tread::~Tread() {
	// empty
}

void Tread::PIDWrite(float output) {
	m_last = output;
	output = Tools::Limit(output, -1.0, 1.0);
	m_front->PIDWrite(output);
	m_back->PIDWrite(output);
}




PidSimpleDrive::PidSimpleDrive (
			SpeedController *leftFront, 
			SpeedController *leftBack, 
			SpeedController *rightFront,
			SpeedController *rightBack,
			Encoder *leftEncoder,
			Encoder *rightEncoder) :
			BaseDrive("PidSimpleDrive"),
			IPidDrive(),
			m_currentMode(Rate) {

	m_leftFront = leftFront;
	m_leftBack = leftBack;
	m_rightFront = rightFront;
	m_rightBack = rightBack;
	m_leftEncoder = leftEncoder;
	m_rightEncoder = rightEncoder;
	
	m_leftEncoder->Start();
	m_rightEncoder->Start();
	
	m_smoothLeftEncoder = new SmoothEncoder(m_leftEncoder, 5);
	m_smoothRightEncoder = new SmoothEncoder(m_rightEncoder, 5);
	
	m_leftTread = new Tread(m_leftFront, m_leftBack);
	m_rightTread = new Tread(m_rightFront, m_rightBack);
	
	m_leftPidRate = new PIDController(
			0, 0, 0,
			m_smoothLeftEncoder,
			m_leftTread);
	m_rightPidRate = new PIDController(
			0, 0, 0,
			m_smoothRightEncoder,
			m_rightTread);
	
	m_leftPidDistance = new PIDController(
			0.1, 0.01, 0.00,
			m_smoothLeftEncoder,
			m_leftTread);
	m_rightPidDistance = new PIDController(
			0.1, 0.01, 0.00,
			m_smoothRightEncoder,
			m_rightTread);
			
	m_leftPidRate->Enable();
	m_rightPidRate->Enable();
	
	AddActuatorToLiveWindow("Left PID Rate", m_leftPidRate);
	AddActuatorToLiveWindow("Right PID Rate", m_rightPidRate);
	AddActuatorToLiveWindow("Left PID Distance", m_leftPidDistance);
	AddActuatorToLiveWindow("Right PID Distance", m_rightPidDistance);
}

PidSimpleDrive::~PidSimpleDrive() {
	// Empty
}
	
void PidSimpleDrive::Drive(float outputMagnitude, float curve) {
	TryToggling(Rate);
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
	m_leftPidRate->SetSetpoint(leftValue);
	m_rightPidRate->SetSetpoint(rightValue);
	SmartDashboard::PutNumber("Left Tread", m_leftTread->m_last);
	SmartDashboard::PutNumber("Right Tread", m_rightTread->m_last);
}

void PidSimpleDrive::ArcadeDrive(float moveValue, float rotateValue) {
	TryToggling(Rate);
	ArcadeDrive(moveValue, rotateValue, false);
}

void PidSimpleDrive::ArcadeDrive(float moveValue, float rotateValue, bool squaredInputs) {
	TryToggling(Rate);
	
}

void PidSimpleDrive::TravelDistance(float distanceInInches) {
	TryToggling(Distance);
}

void PidSimpleDrive::Rotate(float degrees) {
	TryToggling(Distance);
}

void PidSimpleDrive::Disable() {
	m_leftPidRate->Disable();
	m_rightPidRate->Disable();
	m_leftPidDistance->Disable();
	m_leftPidDistance->Disable();
	m_leftTread->PIDWrite(0);
	m_rightTread->PIDWrite(0);
}

void PidSimpleDrive::Enable() {
	m_leftPidRate->Enable();
	m_rightPidRate->Enable();
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
		m_leftPidDistance->Disable();
		m_rightPidDistance->Disable();
		m_leftPidRate->Enable();
		m_rightPidRate->Enable();
		m_currentMode = Rate;
		break;
	case Distance:
		m_leftPidRate->Disable();
		m_rightPidRate->Disable();
		m_leftPidDistance->Enable();
		m_rightPidDistance->Enable();
		m_currentMode = Distance;
		break;
	default:
		SmartDashboard::PutString("PidSimpleDrive Toggling error", "Unknown option");
	}
}

void PidSimpleDrive::AdjustRatePid(float lp, float li, float ld, float rp, float ri, float rd) {
	m_leftPidRate->SetPID(lp, li, ld);
	m_rightPidRate->SetPID(rp, ri, rd);
}
	
