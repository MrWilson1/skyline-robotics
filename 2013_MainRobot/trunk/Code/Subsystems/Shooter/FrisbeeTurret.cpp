#include "FrisbeeTurret.h"

FrisbeeTurret::Base::Base(const char *name) :
		BaseSubsystem(name) {
	// Empty
}
FrisbeeTurret::Base::~Base() {
	// Empty
}



FrisbeeTurret::Simple::Simple(SpeedController *motor, const char *name) :
		FrisbeeTurret::Base(name) {
	m_motor = motor; 
	AddActuatorToLiveWindow("Motor", m_motor);
}

FrisbeeTurret::Simple::~Simple() {
	//empty
}

void FrisbeeTurret::Simple::SetSpeed(float speed){
	m_motor->Set(speed); 
}

void FrisbeeTurret::Simple::Stop(){
	m_motor->Set(0);
}


	


FrisbeeTurret::Guarded::Guarded(
		SpeedController *motor, 
		DigitalInput *switch1,
		DigitalInput *switch2,
		const char *name) :
		FrisbeeTurret::Base(name) {
	m_motor = motor;
	m_switch1 = switch1;
	m_switch2 = switch2;
	
	AddActuatorToLiveWindow("Motor", m_motor);
	AddSensorToLiveWindow("Switch 1", m_switch1);
	AddSensorToLiveWindow("Switch 2", m_switch2);
}

FrisbeeTurret::Guarded::~Guarded() {
	//empty
}
void FrisbeeTurret::Guarded::SetSpeed(float speed){
	if (m_switch1->Get() && speed > 0.0) {
		m_motor->Set(0.0);
		return;
	}
	if (m_switch2->Get() && speed < 0.0) {
		m_motor->Set(0.0);
		return;
	}
		
	m_motor->Set(speed);
}

void FrisbeeTurret::Guarded::Stop(){
	m_motor->Set(0);
}



TurretPosition::Base::Base(const char *name) :
	BaseSubsystem(name) {
	// empty
}

TurretPosition::Base::~Base() {
	// empty
}



TurretPosition::EncoderAngle::EncoderAngle(Encoder *encoder, float home, const char *name) :
		Base(name),
		m_home(home){
	m_encoder = encoder;
	AddSensorToLiveWindow("encoder", m_encoder);
}

TurretPosition::EncoderAngle::~EncoderAngle() {
	// empty
}

TurretPosition::Position TurretPosition::EncoderAngle::GetPosition() {
	float angle = m_encoder->Get();
	if (Tools::IsWithinRange(angle, m_home, 4)) {
		return TurretPosition::kCenter;
	} else if (angle > (m_home + 4)) {
		return TurretPosition::kRight;
	} else if (angle < (m_home - 4)) {
		return TurretPosition::kLeft;
	} else {
		return TurretPosition::kError;
	}
}

float TurretPosition::EncoderAngle::GetAngle() {
	return m_encoder->Get();
}

void TurretPosition::EncoderAngle::SetHome(float angle) {
	m_home = angle;
}




TurretPosition::TwoLimitSwitches::TwoLimitSwitches(DigitalInput *leftSwitch, DigitalInput *rightSwitch) :
		TurretPosition::Base("TurretPosition::TwoLimitSwitches") {
	m_leftSwitch = leftSwitch;
	m_rightSwitch = rightSwitch;
	AddSensorToLiveWindow("left switch", m_leftSwitch);
	AddSensorToLiveWindow("right switch", m_rightSwitch);
}

TurretPosition::TwoLimitSwitches::~TwoLimitSwitches() {
	// empty
}

TurretPosition::Position TurretPosition::TwoLimitSwitches::GetPosition() {
	if (m_leftSwitch->Get() and m_rightSwitch->Get()) {
		return TurretPosition::kCenter;
	} else if (m_leftSwitch->Get()) {
		return TurretPosition::kLeft;
	} else if (m_rightSwitch->Get()) {
		return TurretPosition::kRight;
	} else {
		return TurretPosition::kError;
	}
}

float TurretPosition::TwoLimitSwitches::GetAngle() {
	return 0;
}

void TurretPosition::TwoLimitSwitches::SetHome(float angle) {
	// do nothing.
}
