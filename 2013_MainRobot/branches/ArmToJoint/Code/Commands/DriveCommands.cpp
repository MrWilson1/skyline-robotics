#include "DriveCommands.h"

TankDriveCommand::TankDriveCommand(BaseDrive *drive, Axis *leftAxis, Axis *rightAxis) :
		SimpleCommand("TankDrive", true) {
	m_drive = drive;
	m_leftAxis = leftAxis;
	m_rightAxis = rightAxis;
	Requires(m_drive);
}

TankDriveCommand::~TankDriveCommand() {
	// Empty
}

void TankDriveCommand::Execute() {
	float left = m_leftAxis->Get();
	float right = m_rightAxis->Get();
	SmartDashboard::PutNumber("TankDriveCommand Left", left);
	SmartDashboard::PutNumber("TankDriveCommand Right", right);
	m_drive->TankDrive(left, right, true);
}


ArcadeDriveCommand::ArcadeDriveCommand(BaseDrive *drive,  Axis *magnitudeAxis, Axis *rotateAxis) :
		SimpleCommand("ArcadeDrive", true) {
	m_drive = drive;
	m_magnitudeAxis = magnitudeAxis;
	m_rotateAxis = rotateAxis;
	Requires(m_drive);
}

ArcadeDriveCommand::~ArcadeDriveCommand() {
	// empty
	
}

void ArcadeDriveCommand::Execute() {
	float magnitude = m_magnitudeAxis->Get();
	float rotate = m_rotateAxis->Get();
	m_drive->ArcadeDrive(magnitude, rotate, true);
}



TravelStraightManualCommand::TravelStraightManualCommand(BaseDrive *drive, Axis *axis) :
		Command("TravelStraightManualCommand") {
	m_drive = drive;
	m_axis = axis;
	Requires(m_drive);
}

TravelStraightManualCommand::~TravelStraightManualCommand() {
	// empty
}

void TravelStraightManualCommand::Initialize() {
	// empty
}

void TravelStraightManualCommand::Execute() {
	float magnitude = m_axis->Get();
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


ToggleTransmissionCommand::ToggleTransmissionCommand(BaseDriveTransmission *transmission) :
		SimpleCommand("ToggleTransmissionCommand", false) {
	m_transmission = transmission;
	Requires(m_transmission);
}

ToggleTransmissionCommand::~ToggleTransmissionCommand() {
	// empty
}

void ToggleTransmissionCommand::Execute() {
	m_transmission->ToggleGear();
}
