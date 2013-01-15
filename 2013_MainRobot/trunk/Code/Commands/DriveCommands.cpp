#include "DriveCommands.h"

TankDriveCommand::TankDriveCommand(BaseDrive *drive, BaseOI *OI) :
		Command("TankDrive") {
	m_drive = drive;
	m_OI = OI;
	Requires(m_drive);
}

TankDriveCommand::~TankDriveCommand() {
	// Empty
}

void TankDriveCommand::Initialize()  {
	// Empty
}

void TankDriveCommand::Execute() {
	TankValues t = m_OI->GetTankValues();
	SmartDashboard::PutNumber("TankDriveCommand Left", t.Left);
	SmartDashboard::PutNumber("TankDriveCommand Right", t.Right);
	m_drive->TankDrive(t.Left, t.Right, true);
}

bool TankDriveCommand::IsFinished() {
	return false; // Runs forever until explicitly silenced.
}

void TankDriveCommand::End() { 
	// Empty
}

void TankDriveCommand::Interrupted() { 
	// Empty
}


ArcadeDriveCommand::ArcadeDriveCommand(BaseDrive *drive, BaseOI *OI) :
		Command("ArcadeDrive") {
	m_drive = drive;
	m_OI = OI;
	Requires(m_drive);
}

ArcadeDriveCommand::~ArcadeDriveCommand() {
	// empty
	
}

void ArcadeDriveCommand::Initialize() {
	// Empty
}

void ArcadeDriveCommand::Execute() {
	ArcadeValues a = m_OI->GetArcadeValues();
	m_drive->ArcadeDrive(a.Move, a.Rotate, true);
}

bool ArcadeDriveCommand::IsFinished() {
	return false; // Runs forever until explicitly silenced.
}

void ArcadeDriveCommand::End() { 
	// Empty
}

void ArcadeDriveCommand::Interrupted() 
{ 
	// Empty
}


Spasm::Spasm(BaseDrive *drive) :
		Command("Spasm"),
		counter(0) {
	m_drive = drive;
	Requires(m_drive);
}

Spasm::~Spasm() {
	// empty
}

void Spasm::Initialize() {
	// empty
}
void Spasm::Execute() {
	SmartDashboard::PutString("Spasm", "Spasming");
	float value = 0.5;
	counter += 1;
	m_drive->TankDrive(-value, value);
}

bool Spasm::IsFinished() {
	if (counter > 100) {
		counter = 0;
		return true;
	} else {
		return false;
	}
}

void Spasm::End() {
	SmartDashboard::PutString("Spasm", "Not Spasming");
}

void Spasm::Interrupted() {
	SmartDashboard::PutString("Spasm", "Interrupted Spasming");
	float value = 0.0;
	counter = 103;
	m_drive->TankDrive(value, value);
}
