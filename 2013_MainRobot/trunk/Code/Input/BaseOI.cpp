#include "BaseOI.h"

BaseOI::BaseOI(const char *name) :
	m_name(name)
{
	// empty
}

BaseOI::~BaseOI()
{
	// empty
}

std::string BaseOI::GetName()
{
	return m_name;
}
/*
void BaseOI::SetupTelop()
{
	SmartDashboard::PutString("BaseOI", "AIII");
	// empty
}

TankValues BaseOI::GetTankValues()
{
	TankValues t;
	t.Left = 0;
	t.Right = 0;
	return t;
}

ArcadeValues BaseOI::GetArcadeValues()
{
	ArcadeValues a;
	a.Move = 0;
	a.Rotate = 0;
	return a;
}

*/
