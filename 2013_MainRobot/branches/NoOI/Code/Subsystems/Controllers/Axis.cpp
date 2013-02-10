#include "Axis.h"

Axis::Axis(Joystick *joystick, int axisNum) :
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
