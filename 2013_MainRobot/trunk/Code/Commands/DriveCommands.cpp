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
	float left = Tools::Deadband(m_leftAxis->Get(), 0.1);
	float right = Tools::Deadband(m_rightAxis->Get(), 0.1);
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




RefreshPidCommand::RefreshPidCommand(IPidDrive *drive, Encoder::PIDSourceParameter pidSource) :
		SimpleCommand("RefreshPidCommand", true),
		m_pidSource(pidSource) {
	m_drive = drive;
	DrivePid pid = m_drive->GetRatePid(); // placeholder until it needs to be replaced.
	if (m_pidSource == Encoder::kRate) {
		pid = m_drive->GetRatePid();
	} else if (m_pidSource == Encoder::kDistance) {
		pid = m_drive->GetDistancePid();
	}
	SmartDashboard::PutNumber("Left Rate P", pid.Left.GetD());
	SmartDashboard::PutNumber("Left Rate I", pid.Left.GetI());
	SmartDashboard::PutNumber("Left Rate D", pid.Left.GetD());
	SmartDashboard::PutNumber("Right Rate P", pid.Right.GetP());
	SmartDashboard::PutNumber("Right Rate I", pid.Right.GetI());
	SmartDashboard::PutNumber("Right Rate D", pid.Right.GetD());
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
	if (m_pidSource == Encoder::kRate) {
		m_drive->AdjustRatePid(lp, li, ld, rp, ri, rd);
	} else if (m_pidSource == Encoder::kDistance) {
		m_drive->AdjustDistancePid(lp, li, ld, rp, ri, rd);
	}
}




TravelDistanceCommand::TravelDistanceCommand(BaseDrive *drive, float distance, Tools::Units unit) :
		Command("TravelDistanceCommand"),
		m_distanceInInches(Tools::ConvertUnits(distance, unit, Tools::kInches)) {
	m_drive = drive;
	Requires(m_drive);
	SmartDashboard::PutNumber("Travel Distance:", 0.0);
}

TravelDistanceCommand::TravelDistanceCommand(BaseDrive *drive, float distanceInInches) :
		Command("TravelDistanceCommand"),
		m_distanceInInches(distanceInInches) {
	m_drive = drive;
	Requires(m_drive);
	SmartDashboard::PutNumber("Travel Distance:", 0.0);
}

TravelDistanceCommand::~TravelDistanceCommand() {
	//empty
}
	
void TravelDistanceCommand::Initialize() {
	m_drive->ResetDistanceAndRotation();
}

void TravelDistanceCommand::Execute() {
	//m_drive->TravelDistance(m_distanceInInches);
	float m_distanceInInches = SmartDashboard::GetNumber("Travel Distance:");
	m_drive->TravelDistance(m_distanceInInches);
}

bool TravelDistanceCommand::IsFinished() {
	//DrivePid pid = dynamic_cast<IPidDrive*>(m_drive)->GetDistancePid(); 
	//float distance = (pid.Left.GetDistance() + pid.Right.GetDistance()) / 2; // take the average
	//return Tools::IsWithinRange(distance, m_distanceInInches, 1);
	return false;
}

void TravelDistanceCommand::End() {
	// empty
}

void TravelDistanceCommand::Interrupted() {
	// empty
}

RotateRobotCommand::RotateRobotCommand(BaseDrive *drive, float degrees) :
		Command("RotateRobotCommand"),
		m_degrees(degrees) {
	m_drive = drive;
}

RotateRobotCommand::~RotateRobotCommand() {
	// empty
}
	
void RotateRobotCommand::Initialize() {
	// empty
}

void RotateRobotCommand::Execute() {
	// todo
}

bool RotateRobotCommand::IsFinished() {
	return true;
}

void RotateRobotCommand::End() {
	// empty
}

void RotateRobotCommand::Interrupted() {
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


ToggleTransmissionSwapPidCommand::ToggleTransmissionSwapPidCommand(BaseDriveTransmission *transmission, 
		IPidDrive *drive, 
		DrivePid lowGearRate, DrivePid lowGearDistance,
		DrivePid highGearRate, DrivePid highGearDistance) :
		SimpleCommand("ToggleTransmissionCommand", true),
		m_lowGearRate(lowGearRate),
		m_lowGearDistance(lowGearDistance),
		m_highGearRate(highGearRate),
		m_highGearDistance(highGearDistance) {
	m_transmission = transmission;
	m_drive = drive;
	Requires(m_transmission);
	Requires(dynamic_cast<BaseDrive *>(m_drive));
}

ToggleTransmissionSwapPidCommand::~ToggleTransmissionSwapPidCommand() {
	// empty
}

void ToggleTransmissionSwapPidCommand::Execute() {
	if (m_transmission->GetCurrentMode() == BaseDriveTransmission::kHighGear) {
		m_drive->SetRatePid(m_highGearRate);
		m_drive->SetDistancePid(m_highGearDistance);
		m_transmission->SetHighGear();
	} else if (m_transmission->GetCurrentMode() == BaseDriveTransmission::kLowGear) {
		m_drive->SetRatePid(m_lowGearRate);
		m_drive->SetDistancePid(m_lowGearDistance);
		m_transmission->SetLowGear();
	}
}
