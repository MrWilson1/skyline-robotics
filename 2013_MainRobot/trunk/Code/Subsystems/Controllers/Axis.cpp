#include "Axis.h"

BaseAxis::BaseAxis() {
	// empty
}

BaseAxis::~BaseAxis() {
	// empty
}

Axis::Axis(Joystick *joystick, int axisNum) :
		BaseAxis(), 
		m_axisNum(axisNum) {
	m_joystick = joystick;
}

Axis::~Axis() {
	// empty
}

float Axis::Get() {
	return m_joystick->GetRawAxis(m_axisNum);
}

Joystick * Axis::GetJoystick() {
	return m_joystick;
}

const int Axis::GetAxisNum() {
	return m_axisNum;
}




TruncatedCurvedAxis::TruncatedCurvedAxis(Joystick *joystick, int axisNum) :
		BaseAxis(), 
		m_axisNum(axisNum) {
	m_joystick = joystick;
}

TruncatedCurvedAxis::~TruncatedCurvedAxis() {
	// empty
}

float TruncatedCurvedAxis::Get() {
	float value = m_joystick->GetRawAxis(m_axisNum);
	value = Tools::Deadband(value, 0.2);
	value = value * value;
	return value;
}

Joystick * TruncatedCurvedAxis::GetJoystick() {
	return m_joystick;
}

const int TruncatedCurvedAxis::GetAxisNum() {
	return m_axisNum;
}
