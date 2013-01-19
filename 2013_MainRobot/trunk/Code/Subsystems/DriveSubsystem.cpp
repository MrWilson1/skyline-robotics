#include "DriveSubsystem.h"

BaseDrive::BaseDrive(const char *name) :
	BaseSubsystem(name) {
	// empty
}

BaseDrive::~BaseDrive() {
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



/**
 * Simply tells the wheels to stop spinning.
 */
void SimpleDrive::StopMoving() {
	float stopValue = 0.0;
	m_robotDrive->TankDrive(stopValue, stopValue);
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
	for (unsigned int i=0; i < m_maxSize; i++) {
		sum += m_history.at(i);
	}
	double average = sum / m_maxSize;
	return average;
}

Encoder * SmoothEncoder::GetEncoder() {
	return m_encoder;
}

void SmoothEncoder::SetMaxSize(unsigned int maxSize) {
	m_maxSize = maxSize;
}



Tread::Tread(SpeedController *front, SpeedController *back) {
	m_front = front;
	m_back = back;
}

Tread::~Tread() {
	// empty
}

void Tread::PIDWrite(float output) {
	Tools::Limit(output, -1.0, 1.0);
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
			BaseDrive("PidSimpleDrive") {

	m_leftFront = leftFront;
	m_leftBack = leftBack;
	m_rightFront = rightFront;
	m_rightBack = rightBack;
	m_leftEncoder = leftEncoder;
	m_rightEncoder = rightEncoder;
	
	m_leftEncoder->Start();
	m_rightEncoder->Start();
	
	/*AddActuatorToLiveWindow("Left front motor", m_leftFront);
	AddActuatorToLiveWindow("Left back motor", m_leftBack);
	AddActuatorToLiveWindow("Right front motor", m_rightFront);
	AddActuatorToLiveWindow("Right back motor", m_rightBack);
	AddSensorToLiveWindow("Left encoder", m_leftEncoder);
	AddSensorToLiveWindow("Right encoder", m_rightEncoder);*/
	
	m_smoothLeftEncoder = new SmoothEncoder(m_leftEncoder, 5);
	m_smoothRightEncoder = new SmoothEncoder(m_rightEncoder, 5);
	
	m_leftTread = new Tread(m_leftFront, m_leftBack);
	m_rightTread = new Tread(m_rightFront, m_rightBack);
	
	m_leftPid = new PIDController(
			0.1, 0.01, 0.00,
			m_smoothLeftEncoder,
			m_leftTread);
	m_rightPid = new PIDController(
			0.1, 0.01, 0.00,
			m_smoothLeftEncoder,
			m_leftTread);
			
	m_leftPid->Enable();
	m_rightPid->Enable();
	
	AddActuatorToLiveWindow("Left PID", m_leftPid);
	AddActuatorToLiveWindow("Right PID", m_rightPid);
}

PidSimpleDrive::~PidSimpleDrive() {
	// Empty
}
	
void PidSimpleDrive::Drive(float outputMagnitude, float curve) {
	
}

void PidSimpleDrive::TankDrive(float leftValue, float rightValue) {
	
}

void PidSimpleDrive::TankDrive(float leftValue, float rightValue, bool squaredInputs) {
	
}

void PidSimpleDrive::ArcadeDrive(float moveValue, float rotateValue) {
	
}

void PidSimpleDrive::ArcadeDrive(float moveValue, float rotateValue, bool squaredInputs) {
	
}

void PidSimpleDrive::TravelDistance(float distanceInInches) {

}

void PidSimpleDrive::Rotate(float degrees) {
	
}

void PidSimpleDrive::StopMoving() {
	
}

void PidSimpleDrive::Brake() {
	
}
	
