#include "FrisbeeTurret.h"

BaseFrisbeeTurret::BaseFrisbeeTurret(const char *name) :
		BaseSubsystem(name) {
	// Empty
}

BaseFrisbeeTurret::~BaseFrisbeeTurret() {
	// Empty
}

/*

PidFrisbeeTurret::PidFrisbeeTurret(
			SpeedController *horizontalMotor, 
			SpeedController *verticalMotor,
			Encoder *horizontalEncoder,
			Encoder *verticalEncoder) :
			BaseFrisbeeTurret("Pid Frisbee Turret"),
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
	
	AddActuatorToLiveWindow<PIDController>("Horizontal PID", m_horizontalPid);
	AddActuatorToLiveWindow<PIDController>("Vertical PID", m_verticalPid);
	
	AddSensorToLiveWindow<Encoder>("Horizontal Encoder", m_horizontalEncoder);
	AddSensorToLiveWindow<Encoder>("Vertical Encoder", m_verticalEncoder);
}

void PidFrisbeeTurret::TurnHorizontal(float degrees) {
	m_offset.XOffset += degrees;
	m_horizontalPid->SetSetpoint(degrees);
}

void PidFrisbeeTurret::TurnVertical(float degrees) {
	m_offset.YOffset += degrees;
	m_verticalPid->SetSetpoint(degrees);
}

void PidFrisbeeTurret::TurnGivenOffset(Offset offset) {
	TurnHorizontal(offset.XOffset);
	TurnVertical(offset.YOffset);
}

Offset PidFrisbeeTurret::ReportCurrentOffset() {
	return m_offset;
}
*/
