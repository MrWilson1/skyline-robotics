#include "ShooterCommands.h"

LoadFrisbeeCommand::LoadFrisbeeCommand(BaseFrisbeeLoader *loader) :
		Command("LoadFrisbee"),
		m_isFinished(false) {
	m_loader = loader;
	Requires(m_loader);
}

LoadFrisbeeCommand::~LoadFrisbeeCommand() {
	// empty
}

void LoadFrisbeeCommand::Initialize() {
	// empty
}

void LoadFrisbeeCommand::Execute() {
	SmartDashboard::PutNumber("Frisbees loaded", m_loader->GetNumberOfFrisbeesLoaded());
	if (m_loader->IsFrisbeePrepared()) {
		m_loader->LoadFrisbee();
		m_isFinished = true;
	} else {
		m_loader->PrepareFrisbee();
	}
}

bool LoadFrisbeeCommand::IsFinished() {
	return m_isFinished;
}

void LoadFrisbeeCommand::End() {
	// empty
}

void LoadFrisbeeCommand::Interrupted() {
	// empty
}

AimTurretCommand::AimTurretCommand(
		BaseFrisbeeAimer *aimer, 
		BaseFrisbeeTurret *turret, 
		Tracking::TargetType desiredTarget,
		float allowedRange) :
		Command("AimTurret"),
		m_isFinished(false),
		m_desiredTarget(desiredTarget),
		m_allowedRange(allowedRange){
	m_aimer = aimer;
	m_turret = turret;
	Requires(m_aimer);
	Requires(m_turret);
}

AimTurretCommand::~AimTurretCommand() {
	// empty
}

void AimTurretCommand::Initialize() {
	// empty
}

void AimTurretCommand::Execute() {
	Tracking::Target target;
	switch (m_desiredTarget) {
	case (Tracking::Low) :
		target = m_aimer->GetLowTarget();
		break;
	case (Tracking::MediumLeft) :
		target = m_aimer->GetMediumLeftTarget();
		break;
	case (Tracking::MediumRight) :
		target = m_aimer->GetMediumRightTarget();
		break;
	case (Tracking::High) :
		target = m_aimer->GetHighTarget();
		break;
	case (Tracking::ClosestDistance) :
		target = m_aimer->GetClosestTargetByDistance();
		break;
	case (Tracking::ClosestOffset) :
		target = m_aimer->GetClosestTargetByOffset();
		break;
	case (Tracking::None) :
	case (Tracking::Unknown) :
	case (Tracking::Test) :
	case (Tracking::Pyramid) :
	default:
		SmartDashboard::PutString("AimTurrentCommand error", "Invalid TargetType");
		m_isFinished = true;
		return;
	}
		
	Tracking::Offset desired = target.ShooterOffset;
	Tracking::Offset current = m_turret->GetCurrentOffset();
	bool isXDone = Tools::IsWithinRange(desired.XOffset, current.XOffset, m_allowedRange);
	bool isYDone = Tools::IsWithinRange(desired.YOffset, current.YOffset, m_allowedRange);
	m_isFinished = isXDone and isYDone;
	if (!m_isFinished) {
		m_turret->TurnGivenOffset(desired);
	}
}

bool AimTurretCommand::IsFinished() {
	return m_isFinished;
}

void AimTurretCommand::End() {
	// empty
}

void AimTurretCommand::Interrupted() {
	// empty
}

FireFrisbeeCommand::FireFrisbeeCommand(BaseFrisbeeShooter *shooter) :
		Command("FireFrisbee") {
	m_shooter = shooter;
	Requires(m_shooter);
}

FireFrisbeeCommand::~FireFrisbeeCommand() {
	// empty
}

void FireFrisbeeCommand::Initialize() {
	// empty
}

void FireFrisbeeCommand::Execute() {
	m_shooter->ShootFrisbee();
}

bool FireFrisbeeCommand::IsFinished() {
	return false;
}

void FireFrisbeeCommand::End() {
	m_shooter->StopFrisbee();
}

void FireFrisbeeCommand::Interrupted() {
	m_shooter->StopFrisbee();
}

EjectFrisbeeCommand::EjectFrisbeeCommand(BaseFrisbeeShooter *shooter) :
		Command("EjectFrisbee") {
	m_shooter = shooter;
	Requires(m_shooter);
}

EjectFrisbeeCommand::~EjectFrisbeeCommand() {
	// empty
}

void EjectFrisbeeCommand::Initialize() {
	// empty
}

void EjectFrisbeeCommand::Execute() {
	m_shooter->EjectFrisbee();
}

bool EjectFrisbeeCommand::IsFinished() {
	return true;
}

void EjectFrisbeeCommand::End() {
	// empty
}

void EjectFrisbeeCommand::Interrupted() {
	// empty
}

LoadAndFireCommand::LoadAndFireCommand(
		BaseFrisbeeLoader *loader, 
		BaseFrisbeeAimer *aimer, 
		BaseFrisbeeTurret *turret, 
		BaseFrisbeeShooter *shooter) :
		CommandGroup("LoadAndFireCommand") {
	AddSequential(new LoadFrisbeeCommand(loader));
	AddSequential(new AimTurretCommand(aimer, turret, Tracking::ClosestOffset, 5));
	AddSequential(new FireFrisbeeCommand(shooter), 5.0);
}

LoadAndFireCommand::~LoadAndFireCommand() {
	// empty
}

/**
 * todo: Make two versions of this: one to manually go to some distance (replace Axis with doubles)
 * and another to control using Axis
 */
ManuallyAdjustTurretCommand::ManuallyAdjustTurretCommand(
		BaseFrisbeeTurret *turret,
		Axis *verticalAxis, 
		Axis *rotateAxis,
		float allowedRange) :
		Command("ManuallyAdjustTurretCommand"),
		m_allowedRange(allowedRange),
		m_isFinished(false) {
	m_turret = turret;
	m_verticalAxis = verticalAxis;
	m_rotateAxis = rotateAxis;
}

ManuallyAdjustTurretCommand::~ManuallyAdjustTurretCommand() {
	// empty
}

void ManuallyAdjustTurretCommand::Initialize() {
	SmartDashboard::PutString(GetName(), "init");
}

void ManuallyAdjustTurretCommand::Execute() {
	SmartDashboard::PutString(GetName(), "execute");
	float rawVertical = Tools::Deadband(m_verticalAxis->Get(), 0.1);
	float rawRotate = Tools::Deadband(m_rotateAxis->Get(), 0.1);
	
	// This might not be a good idead
	/*if (rawVertical == 0 and rawRotate == 0) {
		m_isFinished = true;
		return;
	}*/
	
	/*Tracking::Offset desired = m_turret->GetCurrentOffset();
	desired.YOffset += Tools::Scale(rawVertical, -1.0, 1.0, -5.0, 5.0);
	desired.XOffset += Tools::Scale(rawRotate, -1.0, 1.0, -5.0, 5.0);
	
	m_turret->TurnGivenOffset(desired);*/
	Tracking::Offset desired;
	desired.XOffset = rawVertical;
	desired.YOffset = rawRotate;
	m_turret->TurnGivenOffset(desired);
}

bool ManuallyAdjustTurretCommand::IsFinished() {
	//return m_isFinished;
	return false;
}

void ManuallyAdjustTurretCommand::End() {
	SmartDashboard::PutString(GetName(), "end");
	// empty
}

void ManuallyAdjustTurretCommand::Interrupted() {
	SmartDashboard::PutString(GetName(), "interrupt");
	// empty
}



ManuallyControlTurretCommand::ManuallyControlTurretCommand(BaseFrisbeeTurret *turret, Axis *verticalAxis, Axis *rotateAxis) :
			Command("ManuallyControlTurretCommand") {
	m_turret = turret;
	m_verticalAxis = verticalAxis;
	m_rotateAxis = rotateAxis;
	Requires(m_turret);
}

ManuallyControlTurretCommand::~ManuallyControlTurretCommand() {
	// empty
}

void ManuallyControlTurretCommand::Initialize() {
	// empty
}

void ManuallyControlTurretCommand::Execute() {
	m_turret->TurnHorizontal(m_rotateAxis->Get());
	m_turret->TurnVertical(m_verticalAxis->Get());
}

bool ManuallyControlTurretCommand::IsFinished() {
	return false;
}

void ManuallyControlTurretCommand::End() {
	// empty
}

void ManuallyControlTurretCommand::Interrupted() {
	// empty
}

