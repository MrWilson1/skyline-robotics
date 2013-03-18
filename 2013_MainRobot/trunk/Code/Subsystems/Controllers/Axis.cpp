#include "Axis.h"

BaseAxis::BaseAxis() {
	// empty
}

BaseAxis::~BaseAxis() {
	// empty
}

Axis::Axis(Joystick *joystick, const int axisNum) :
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




TruncatedCurvedAxis::TruncatedCurvedAxis(Joystick *joystick, const int axisNum) :
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
	if (value < 0) {
		value = -(value * value);
	} else {
		value = value * value;
	}
	return value;
}

Joystick * TruncatedCurvedAxis::GetJoystick() {
	return m_joystick;
}

const int TruncatedCurvedAxis::GetAxisNum() {
	return m_axisNum;
}




CustomRangeAxis::CustomRangeAxis(Joystick *joystick, const int axisNum, 
								 const float minRange, const float maxRange) :
		BaseAxis(), 
		m_axisNum(axisNum),
		m_minRange(minRange), 
		m_maxRange(maxRange) {
	m_joystick = joystick;
}

CustomRangeAxis::~CustomRangeAxis() {
	// empty
}

float CustomRangeAxis::Get() {
	float raw = m_joystick->GetRawAxis(m_axisNum); 
	return Tools::Scale(raw, -1.0, 1.0, m_minRange, m_maxRange);
}

Joystick * CustomRangeAxis::GetJoystick() {
	return m_joystick;
}

const int CustomRangeAxis::GetAxisNum() {
	return m_axisNum;
}
