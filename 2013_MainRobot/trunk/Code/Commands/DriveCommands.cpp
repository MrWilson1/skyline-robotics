#include "DriveCommands.h"

TankDrive::TankDrive(BaseDrive *drive, BaseOI *OI) :
		Command("TankDrive")
{
	m_drive = drive;
	m_OI = OI;
}

TankDrive::~TankDrive()
{
	// Empty
}

void TankDrive::Initialize() 
{
	// Empty
}

void TankDrive::Execute()
{
	TankValues t = m_OI->GetTankValues();
	m_drive->TankDrive(t.Left, t.Right, true);
}

bool TankDrive::IsFinished()
{
	return false; // Runs forever until explicitly silenced.
}

void TankDrive::End() 
{ 
	// Empty
}

void TankDrive::Interrupted() 
{ 
	// Empty
}


ArcadeDrive::ArcadeDrive(BaseDrive *drive, BaseOI *OI) :
		Command("ArcadeDrive")
{
	m_drive = drive;
	m_OI = OI;
}

void ArcadeDrive::Initialize() 
{
	// Empty
}

void ArcadeDrive::Execute()
{
	ArcadeValues a = m_OI->GetArcadeValues();
	m_drive->ArcadeDrive(a.Move, a.Rotate, true);
}

bool ArcadeDrive::IsFinished()
{
	return false; // Runs forever until explicitly silenced.
}

void ArcadeDrive::End() 
{ 
	// Empty
}

void ArcadeDrive::Interrupted() 
{ 
	// Empty
}


Spasm::Spasm(BaseDrive *drive) :
		Command("Spasm")
{
	m_drive = drive;
}

Spasm::~Spasm()
{
	// empty
}

void Spasm::Initialize()
{
	// empty
}
void Spasm::Execute()
{
	float value = 0.5;
	m_drive->TankDrive(-value, value);
}

bool Spasm::IsFinished()
{
	return true;
}

void Spasm::End()
{
	// empty
}

void Spasm::Interrupted()
{
	float value = 0.0;
	m_drive->TankDrive(value, value);
}
