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


TravelStraightManualCommand::TravelStraightManualCommand(BaseDrive *drive, BaseOI *oi, PreferredAxis preferredAxis) :
		Command("TravelStraightManualCommand"),
		m_preferredAxis(preferredAxis) {
	m_drive = drive;
	m_oi = oi;
	Requires(m_drive);
}

TravelStraightManualCommand::~TravelStraightManualCommand() {
	// empty
}

void TravelStraightManualCommand::Initialize() {
	// empty
}

void TravelStraightManualCommand::Execute() {
	TankValues t = m_oi->GetTankValues();
	float magnitude;
	switch(m_preferredAxis) {
	case Left:
		magnitude = t.Left;
		break;
	case Right:
		magnitude = t.Right;
		break;
	case Average:
		magnitude = (t.Left + t.Right) / 2;
	}
	m_drive->TankDrive(magnitude, magnitude);
}

bool TravelStraightManualCommand::IsFinished() {
	return false;
}

void TravelStraightManualCommand::End() {
	// empty
}

void TravelStraightManualCommand::Interrupted() {
	// empty
}


RefreshPidCommand::RefreshPidCommand(IPidDrive *drive) :
		SimpleCommand("RefreshPidCommand", false) {
	m_drive = drive;
}

RefreshPidCommand::~RefreshPidCommand() {
	// empty
}

void RefreshPidCommand::Execute() {
	float lp = SmartDashboard::GetNumber("Left Rate P");
	float li = SmartDashboard::GetNumber("Left Rate I");
	float ld = SmartDashboard::GetNumber("Left Rate D");
	float rp = SmartDashboard::GetNumber("Right Rate P");
	float ri = SmartDashboard::GetNumber("Right Rate I");
	float rd = SmartDashboard::GetNumber("Right Rate D");
	m_drive->AdjustRatePid(lp, li, ld, rp, ri, rd);
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



TravelDistanceCommand::TravelDistanceCommand(BaseDrive *drive, float distance, Tools::Units unit) :
		Command("TravelDistanceCommand"),
		m_distanceInInches(Tools::ConvertUnits(distance, unit, Tools::kInches)) {
	m_drive = drive;
}

TravelDistanceCommand::TravelDistanceCommand(BaseDrive *drive, float distanceInInches) :
		Command("TravelDistanceCommand"),
		m_distanceInInches(distanceInInches) {
	m_drive = drive;
	Requires(m_drive);
}


TravelDistanceCommand::~TravelDistanceCommand() {
	//empty
}
	
void TravelDistanceCommand::Initialize() {
	m_drive->ResetDistanceAndRotation();
}

void TravelDistanceCommand::Execute() {
	m_drive->TravelDistance(m_distanceInInches);
}

bool TravelDistanceCommand::IsFinished() {
	return true;
}

void TravelDistanceCommand::End() {
	// empty
}

void TravelDistanceCommand::Interrupted() {
	// empty
}
