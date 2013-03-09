#include "FrisbeeTurret.h"


BaseAxisFrisbeeTurret::BaseAxisFrisbeeTurret(const char *name) :
		BaseSubsystem(name) {
	// Empty
}
BaseAxisFrisbeeTurret::~BaseAxisFrisbeeTurret() {
	// Empty
}



SimpleAxisFrisbeeTurret::SimpleAxisFrisbeeTurret(SpeedController *motor) :
		BaseAxisFrisbeeTurret("BaseAxisFrisbeeTurret"),
		k_highSpeed(0.8),
		k_mediumSpeed(0.5),
		k_lowSpeed(0.3) {
	m_motor = motor; 
	AddActuatorToLiveWindow("Motor", m_motor);
}

SimpleAxisFrisbeeTurret::~SimpleAxisFrisbeeTurret() {
	//empty
}

void SimpleAxisFrisbeeTurret::SetMotor(float speed){
	m_motor->Set(speed); 
}

void SimpleAxisFrisbeeTurret::Stop(){
	m_motor->Set(0);
}

bool SimpleAxisFrisbeeTurret::ShouldTurretStop(){
	return false; 
}

void SimpleAxisFrisbeeTurret::TurnGivenOffset(Tracking::Offset offset, double direction, double upperBound, double middleBound, double lowerBound) {
	if (Tools::IsWithinBounds(upperBound, middleBound, fabs(offset.XOffset))) {
		SetMotor(direction * k_highSpeed);
	} else if (Tools::IsWithinBounds(middleBound, lowerBound, fabs(offset.XOffset))) {
		SetMotor(direction * k_mediumSpeed);
	} else if (Tools::IsWithinBounds(lowerBound, 0, fabs(offset.XOffset))) {
		SetMotor(direction * k_lowSpeed);
	}
}


GuardedAxisFrisbeeTurret::GuardedAxisFrisbeeTurret( SpeedController *motor, 
		 DigitalInput *switch1,
		 DigitalInput *switch2) :
		BaseAxisFrisbeeTurret("GuardedAxisFrsibeeTurret"),
		k_highSpeed(0.8),
		k_mediumSpeed(0.5),
		k_lowSpeed(0.3) {
	m_motor = motor;
	m_switch1 = switch1;
	m_switch2 = switch2;
	AddActuatorToLiveWindow("Motor", m_motor);
	AddSensorToLiveWindow("Switch 1", m_switch1);
	AddSensorToLiveWindow("Switch 2", m_switch2);
}

GuardedAxisFrisbeeTurret::~GuardedAxisFrisbeeTurret() {
	//empty
}
void GuardedAxisFrisbeeTurret::SetMotor(float speed){
	if (ShouldTurretStop()){
		Stop();
	} else {
		m_motor->Set(speed);
	}
}

void GuardedAxisFrisbeeTurret::Stop(){
	m_motor->Set(0);
}

bool GuardedAxisFrisbeeTurret::ShouldTurretStop(){
	return !m_switch1->Get() || !m_switch2->Get(); 
}

void GuardedAxisFrisbeeTurret::TurnGivenOffset(Tracking::Offset offset, double direction, double upper, double middle, double lower) {
	if (Tools::IsWithinBounds(upper, middle, fabs(offset.XOffset))) {
		SetMotor(direction * k_highSpeed);
	} else if (Tools::IsWithinBounds(middle, lower, fabs(offset.XOffset))) {
		SetMotor(direction * k_mediumSpeed);
	} else if (Tools::IsWithinBounds(lower, 0, fabs(offset.XOffset))) {
		SetMotor(direction * k_lowSpeed);
	}
}

////////////////////////////////

BaseFrisbeeTurret::BaseFrisbeeTurret(const char *name) :
		BaseSubsystem(name) {
	// Empty
}

BaseFrisbeeTurret::~BaseFrisbeeTurret() {
	// Empty
}

SimpleFrisbeeTurret::SimpleFrisbeeTurret(
			SpeedController *horizontalMotor, 
			SpeedController *verticalMotor) : 
			BaseFrisbeeTurret("SimpleFrisbeeTurret"),
			m_offset(0, 0) {
	m_horizontalMotor = horizontalMotor;
	m_lateralMotor = verticalMotor;
	
	AddActuatorToLiveWindow("Horizontal", m_horizontalMotor);
	AddActuatorToLiveWindow("Vertical", m_lateralMotor);
}

SimpleFrisbeeTurret::~SimpleFrisbeeTurret() {
	// empty
}

void SimpleFrisbeeTurret::TurnHorizontal(float speed) {
	//m_offset.XOffset += speed;
	SmartDashboard::PutNumber(GetName() + std::string(" Horizontal"), speed);
	m_horizontalMotor->Set(speed);
}

void SimpleFrisbeeTurret::TurnVertical(float speed) {
	//m_offset.YOffset += speed;
	SmartDashboard::PutNumber(GetName() + std::string(" Vertical"), speed);
	m_lateralMotor->Set(speed);
}

void SimpleFrisbeeTurret::TurnGivenOffset(Tracking::Offset offset) {
	TurnHorizontal(offset.XOffset);
	TurnVertical(offset.YOffset);
}

Tracking::Offset SimpleFrisbeeTurret::GetCurrentOffset() {
	return m_offset;
}

GuardedFrisbeeTurret::GuardedFrisbeeTurret(
			SpeedController *horizontalMotor, 
			SpeedController *verticalMotor,
			DigitalInput *leftSwitch,
			DigitalInput *rightSwitch,
			DigitalInput *topSwitch,
			DigitalInput *bottomSwitch) : 
			BaseFrisbeeTurret("GuardedFrisbeeTurret"),
			m_offset(0, 0) {
	m_horizontalMotor = horizontalMotor;
	m_lateralMotor = verticalMotor;
	m_leftSwitch = leftSwitch;
	m_rightSwitch = rightSwitch;
	m_topSwitch = topSwitch;
	m_bottomSwitch = bottomSwitch;
	
	AddActuatorToLiveWindow("Horizontal", m_horizontalMotor);
	AddActuatorToLiveWindow("Vertical", m_lateralMotor);
	AddSensorToLiveWindow("Left limit switch", m_leftSwitch);
	AddSensorToLiveWindow("Right limit switch", m_rightSwitch);
	AddSensorToLiveWindow("Top limit switch", m_topSwitch);
	AddSensorToLiveWindow("Bottom limit switch", m_bottomSwitch);
}

GuardedFrisbeeTurret::~GuardedFrisbeeTurret() {
	// empty
}

void GuardedFrisbeeTurret::TurnHorizontal(float speed) {
	SmartDashboard::PutNumber(GetName() + std::string(" Horizontal"), speed);
	// assuming that limit switch is "True" when hit
	if (!m_rightSwitch || !m_leftSwitch) {
		m_horizontalMotor->Set(0.0);
	} else {
		m_horizontalMotor->Set(speed);
	}
}

void GuardedFrisbeeTurret::TurnVertical(float speed) {
	//m_offset.YOffset += speed;
	SmartDashboard::PutNumber(GetName() + std::string(" Vertical"), speed);
	if (!m_topSwitch || !m_bottomSwitch) {
		m_lateralMotor->Set(0.0);
	} else {
		m_lateralMotor->Set(speed);
	}
}

void GuardedFrisbeeTurret::TurnGivenOffset(Tracking::Offset offset) {
	TurnHorizontal(offset.XOffset);
	TurnVertical(offset.YOffset);
}

Tracking::Offset GuardedFrisbeeTurret::GetCurrentOffset() {
	return m_offset;
}

PidFrisbeeTurret::PidFrisbeeTurret(
			SpeedController *horizontalMotor, 
			SpeedController *verticalMotor,
			Encoder *horizontalEncoder,
			Encoder *verticalEncoder) :
			BaseFrisbeeTurret("PidFrisbeeTurret"),
			m_offset(0.0f, 0.0f) {
	m_horizontalMotor = horizontalMotor;
	m_verticalMotor = verticalMotor;
	m_horizontalEncoder = horizontalEncoder;
	m_verticalEncoder = verticalEncoder;
	
	m_horizontalEncoder->SetPIDSourceParameter(m_horizontalEncoder->kDistance);
	m_verticalEncoder->SetPIDSourceParameter(m_verticalEncoder->kDistance);
	
	m_horizontalPid = new PIDController(
			0.1, 0.01, 0, 
			m_horizontalEncoder, 
			m_horizontalMotor);
	
	m_verticalPid = new PIDController(
			0.1, 0.01, 0,
			m_verticalEncoder,
			m_verticalMotor);
	
	m_horizontalPid->Enable();
	m_verticalPid->Enable();
	
	AddActuatorToLiveWindow("Horizontal PID", m_horizontalPid);
	AddActuatorToLiveWindow("Vertical PID", m_verticalPid);
	
	AddSensorToLiveWindow("Horizontal Encoder", m_horizontalEncoder);
	AddSensorToLiveWindow("Vertical Encoder", m_verticalEncoder);
}

PidFrisbeeTurret::~PidFrisbeeTurret() {
	delete m_horizontalPid;
	delete m_verticalPid;
}

void PidFrisbeeTurret::TurnHorizontal(float degrees) {
	m_offset.XOffset += degrees;
	m_horizontalPid->SetSetpoint(degrees);
}

void PidFrisbeeTurret::TurnVertical(float degrees) {
	m_offset.YOffset += degrees;
	m_verticalPid->SetSetpoint(degrees);
}

void PidFrisbeeTurret::TurnGivenOffset(Tracking::Offset offset) {
	TurnHorizontal(offset.XOffset);
	TurnVertical(offset.YOffset);
}

Tracking::Offset PidFrisbeeTurret::GetCurrentOffset() {
	return m_offset;
}

