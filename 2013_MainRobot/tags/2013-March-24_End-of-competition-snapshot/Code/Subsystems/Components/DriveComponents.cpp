#include "DriveComponents.h"


Tread::Tread(SpeedController *motor) :
		PIDOutput(),
		LiveWindowSendable(),
		ITableListener(),
		m_speed(0),
		m_direction(kForward) {
	m_motors.push_back(motor);
	m_table = NULL;
}
	
Tread::Tread(SpeedController *front, SpeedController *back) :
		PIDOutput(),
		LiveWindowSendable(),
		ITableListener(),
		m_speed(0),
		m_direction(kForward) {
	m_motors.push_back(front);
	m_motors.push_back(back);
	m_table = NULL;
}

Tread::Tread(std::vector<SpeedController*> motors) :
		PIDOutput(),
		LiveWindowSendable(),
		ITableListener(),
		m_speed(0),
		m_direction(kForward) {
	m_motors = motors;
	m_table = NULL;
}

Tread::~Tread() {
	// empty
}

void Tread::SetSpeed(double speed) {
	m_speed = Tools::Limit(speed, -1.0, 1.0);
	std::vector<SpeedController*>::iterator it;
	for (it = m_motors.begin(); it != m_motors.end(); ++it) {
		// `it` is a pointer to the item inside the vector. However,
		// the item itself is a pointer. So even though `it` is already
		// a pointer, we need to dereference itself again so we can
		// properly use SpeedController's methods.
		(*it)->Set(m_speed * m_direction);
	}
}

Tread::Direction Tread::GetDirection() {
	return m_direction;
}

void Tread::SetDirection(Tread::Direction direction) {
	m_direction = direction;
}
	
double Tread::GetSpeed() {
	return m_speed;
}

void Tread::PIDWrite(float output) {
	m_speed = Tools::Limit(output, -1.0, 1.0);
	std::vector<SpeedController*>::iterator it;
	for (it = m_motors.begin(); it != m_motors.end(); ++it) {
		(*it)->PIDWrite(m_speed * m_direction);
	}
}

void Tread::InitTable(ITable *subtable) {
	if (m_table != NULL) {
		// Replace the existing table
		m_table->RemoveTableListener(this);
	}
	m_table = subtable;
	UpdateTable();
}

ITable* Tread::GetTable() {
	return m_table;
}

std::string Tread::GetSmartDashboardType() {
	// The SmartDashboardType must correspond to the ones in this list:
	// http://firstforge.wpi.edu/integration/viewcvs/viewcvs.cgi/trunk/smartdashboard/src/edu/wpi/first/smartdashboard/types/named/?root=smart_dashboard&system=exsy1002
	// The names don't seem to precisely correspond to the values listed above,
	// so I would recommend you look through the WPILib source code to 
	// find some component similar to the one you're making, and 
	// copy their SmartDashboardType.
	return "Speed Controller";
}

void Tread::UpdateTable() {
	if (m_table != NULL) {
		m_table->PutNumber(kSpeedLabel, GetSpeed());
		m_table->PutNumber(kDirectionLabel, GetDirection());
	}
}

void Tread::StartLiveWindowMode() {
	SetSpeed(0);
	m_table->AddTableListener(this, true);
}

void Tread::StopLiveWindowMode() {
	SetSpeed(0);
	m_table->RemoveTableListener(this);
}

void Tread::ValueChanged(
		ITable *source, 
		const std::string &key, 
		EntryValue value, 
		bool isNew) {
	if (isNew) {
		return;
	}
	
	if (key == kSpeedLabel) {
		SetSpeed(value.f);
	} else if (key == kDirectionLabel) {
		if (value.f > 0) {
			m_direction = kForward;
		} else if (value.f < 0) {
			m_direction = kReverse;
		}
	}
}
	
