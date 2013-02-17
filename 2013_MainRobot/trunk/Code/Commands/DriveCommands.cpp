#include "DriveCommands.h"

TankDriveCommand::TankDriveCommand(BaseDrive *drive, Axis *leftAxis, Axis *rightAxis) :
		SimpleCommand("TankDrive", false) {
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
	double moveValue = m_magnitudeAxis->Get();
	double rotateValue = m_rotateAxis->Get();
	double leftMotorOutput;
	double rightMotorOutput;
	if (moveValue > 0.0) {
		if (rotateValue > 0.0) {
			leftMotorOutput = moveValue - rotateValue;
			rightMotorOutput = max(moveValue, rotateValue);
		} else {
			leftMotorOutput = max(moveValue, -rotateValue);
			rightMotorOutput = moveValue + rotateValue;
		}
	} else {
		if (rotateValue > 0.0) {
			leftMotorOutput = - max(-moveValue, rotateValue);
			rightMotorOutput = moveValue + rotateValue;
		} else {
			leftMotorOutput = moveValue - rotateValue;
			rightMotorOutput = - max(-moveValue, -rotateValue);
		}
	}
	m_drive->TankDrive(leftMotorOutput, rightMotorOutput);
}



TravelStraightManualCommand::TravelStraightManualCommand(BaseDrive *drive, Axis *axis) :
		SimpleCommand("TravelStraightManualCommand", false) {
	m_drive = drive;
	m_axis = axis;
	Requires(m_drive);
}

TravelStraightManualCommand::~TravelStraightManualCommand() {
	// empty
}

void TravelStraightManualCommand::Execute() {
	float magnitude = m_axis->Get();
	m_drive->TankDrive(magnitude, magnitude);
}




RefreshPidCommand::RefreshPidCommand(IPidDrive *drive) :
		SimpleCommand("RefreshPidCommand", true) {
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




TravelDistanceCommand::TravelDistanceCommand(BaseDrive *drive, float distance, Tools::Units unit) :
		Command("TravelDistanceCommand"),
		m_distanceInInches(Tools::ConvertUnits(distance, unit, Tools::kInches)) {
	m_drive = drive;
	Requires(m_drive);
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
		SimpleCommand("ToggleTransmissionCommand", true) {
	m_transmission = transmission;
	Requires(m_transmission);
}

ToggleTransmissionCommand::~ToggleTransmissionCommand() {
	// empty
}

void ToggleTransmissionCommand::Execute() {
	m_transmission->ToggleGear();
}
