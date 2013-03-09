#include "DriveCommands.h"

DriveCommand::TankDrive::TankDrive(Drive::Base *drive, Axis *leftAxis, Axis *rightAxis) :
		SimpleCommand("DriveCommand::TankDrive", false) {
	m_drive = drive;
	m_leftAxis = leftAxis;
	m_rightAxis = rightAxis;
	Requires(m_drive);
}

DriveCommand::TankDrive::~TankDrive() {
	// Empty
}

void DriveCommand::TankDrive::Execute() {
	float left = Tools::Deadband(m_leftAxis->Get(), 0.1);
	float right = Tools::Deadband(m_rightAxis->Get(), 0.1);
	SmartDashboard::PutNumber("DriveCommand::TankDrive Left", left);
	SmartDashboard::PutNumber("DriveCommand::TankDrive Right", right);
	m_drive->TankDrive(left, right, true);
}


DriveCommand::ArcadeDrive::ArcadeDrive(Drive::Base *drive,  Axis *magnitudeAxis, Axis *rotateAxis) :
		SimpleCommand("DriveCommand::ArcadeDrive", true) {
	m_drive = drive;
	m_magnitudeAxis = magnitudeAxis;
	m_rotateAxis = rotateAxis;
	Requires(m_drive);
}

DriveCommand::ArcadeDrive::~ArcadeDrive() {
	// empty
	
}

void DriveCommand::ArcadeDrive::Execute() {
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



DriveCommand::TravelStraightManual::TravelStraightManual(Drive::Base *drive, Axis *axis) :
		SimpleCommand("DriveCommand::TravelStraightManual", false) {
	m_drive = drive;
	m_axis = axis;
	Requires(m_drive);
}

DriveCommand::TravelStraightManual::~TravelStraightManual() {
	// empty
}

void DriveCommand::TravelStraightManual::Execute() {
	float magnitude = m_axis->Get();
	m_drive->TankDrive(magnitude, magnitude);
}




DriveCommand::RefreshPid::RefreshPid(Drive::IPid *drive, Encoder::PIDSourceParameter pidSource) :
		SimpleCommand("DriveCommand::RefreshPid", true),
		m_pidSource(pidSource) {
	m_drive = drive;
	Drive::DrivePid pid = m_drive->GetRatePid(); // placeholder until it needs to be replaced.
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

DriveCommand::RefreshPid::~RefreshPid() {
	// empty
}

void DriveCommand::RefreshPid::Execute() {
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




DriveCommand::TravelDistance::TravelDistance(Drive::Base *drive, float distance, Tools::Units unit) :
		Command("DriveCommand::TravelDistance"),
		m_distanceInInches(Tools::ConvertUnits(distance, unit, Tools::kInches)) {
	m_drive = drive;
	Requires(m_drive);
	SmartDashboard::PutNumber("Travel Distance:", 0.0);
}

DriveCommand::TravelDistance::TravelDistance(Drive::Base *drive, float distanceInInches) :
		Command("DriveCommand::TravelDistance"),
		m_distanceInInches(distanceInInches) {
	m_drive = drive;
	Requires(m_drive);
	SmartDashboard::PutNumber("Travel Distance:", 0.0);
}

DriveCommand::TravelDistance::~TravelDistance() {
	//empty
}
	
void DriveCommand::TravelDistance::Initialize() {
	m_drive->ResetDistanceAndRotation();
}

void DriveCommand::TravelDistance::Execute() {
	//m_drive->TravelDistance(m_distanceInInches);
	float m_distanceInInches = SmartDashboard::GetNumber("Travel Distance:");
	m_drive->TravelDistance(m_distanceInInches);
}

bool DriveCommand::TravelDistance::IsFinished() {
	//Drive::DrivePid pid = dynamic_cast<Drive::IPid*>(m_drive)->GetDistancePid(); 
	//float distance = (pid.Left.GetDistance() + pid.Right.GetDistance()) / 2; // take the average
	//return Tools::IsWithinRange(distance, m_distanceInInches, 1);
	return false;
}

void DriveCommand::TravelDistance::End() {
	// empty
}

void DriveCommand::TravelDistance::Interrupted() {
	// empty
}

DriveCommand::RotateRobot::RotateRobot(Drive::Base *drive, float degrees) :
		Command("DriveCommand::RotateRobot"),
		m_degrees(degrees) {
	m_drive = drive;
}

DriveCommand::RotateRobot::~RotateRobot() {
	// empty
}
	
void DriveCommand::RotateRobot::Initialize() {
	// empty
}

void DriveCommand::RotateRobot::Execute() {
	// todo
}

bool DriveCommand::RotateRobot::IsFinished() {
	return true;
}

void DriveCommand::RotateRobot::End() {
	// empty
}

void DriveCommand::RotateRobot::Interrupted() {
	// empty
}


DriveCommand::ToggleTransmission::ToggleTransmission(DriveTransmission::Base *transmission) :
		SimpleCommand("DriveCommand::ToggleTransmission", true) {
	m_transmission = transmission;
	Requires(m_transmission);
}

DriveCommand::ToggleTransmission::~ToggleTransmission() {
	// empty
}

void DriveCommand::ToggleTransmission::Execute() {
	m_transmission->ToggleGear();
}


DriveCommand::ToggleTransmissionSwapPid::ToggleTransmissionSwapPid(
		DriveTransmission::Base *transmission, 
		Drive::IPid *drive, 
		Drive::DrivePid lowGearRate, Drive::DrivePid lowGearDistance,
		Drive::DrivePid highGearRate, Drive::DrivePid highGearDistance) :
		SimpleCommand("DriveCommand::ToggleTransmission", true),
		m_lowGearRate(lowGearRate),
		m_lowGearDistance(lowGearDistance),
		m_highGearRate(highGearRate),
		m_highGearDistance(highGearDistance) {
	m_transmission = transmission;
	m_drive = drive;
	Requires(m_transmission);
	Requires(dynamic_cast<Drive::Base *>(m_drive));
}

DriveCommand::ToggleTransmissionSwapPid::~ToggleTransmissionSwapPid() {
	// empty
}

void DriveCommand::ToggleTransmissionSwapPid::Execute() {
	if (m_transmission->GetCurrentMode() == DriveTransmission::kHighGear) {
		m_drive->SetRatePid(m_highGearRate);
		m_drive->SetDistancePid(m_highGearDistance);
		m_transmission->SetHighGear();
	} else if (m_transmission->GetCurrentMode() == DriveTransmission::kLowGear) {
		m_drive->SetRatePid(m_lowGearRate);
		m_drive->SetDistancePid(m_lowGearDistance);
		m_transmission->SetLowGear();
	}
}
