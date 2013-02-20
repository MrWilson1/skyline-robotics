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
	case (Tracking::MiddleLeft) :
		target = m_aimer->GetMiddleLeftTarget();
		break;
	case (Tracking::MiddleRight) :
		target = m_aimer->GetMiddleRightTarget();
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


LoadAndFireCommand::LoadAndFireCommand(
		BaseFrisbeeLoader *loader, 
		BaseFrisbeeAimer *aimer, 
		BaseFrisbeeTurret *turret, 
		BaseFrisbeeShooter *shooter) :
		CommandGroup("LoadAndFireCommand") {
	AddParallel(new FireFrisbeeCommand(shooter), 5.0);
	AddSequential(new LoadFrisbeeCommand(loader));
	AddSequential(new AimTurretCommand(aimer, turret, Tracking::ClosestOffset, 5));
}

LoadAndFireCommand::~LoadAndFireCommand() {
	// empty
}

/**
 * todo: Make two versions of this: one to manually go to some distance (replace Axis with doubles)
 * and another to control using Axis
 */
AdjustTurretCommand::AdjustTurretCommand(
		BaseFrisbeeTurret *turret,
		double rotateOffset,
		double verticalOffset,
		float allowedRange) :
		Command("AdjustTurretCommand"),
		m_rotateOffset(rotateOffset),
		m_verticalOffset(verticalOffset),
		m_allowedRange(allowedRange),
		m_isFinished(false) {
	m_turret = turret;
	
	Requires(m_turret);
}

AdjustTurretCommand::~AdjustTurretCommand() {
	// empty
}

void AdjustTurretCommand::Initialize() {
	// empty
}

void AdjustTurretCommand::Execute() {
	Tracking::Offset desired = m_turret->GetCurrentOffset();
	desired.XOffset += m_rotateOffset - desired.XOffset;
	desired.YOffset += m_verticalOffset - desired.YOffset;
	
	bool isXClose = Tools::IsWithinRange(desired.XOffset, 0, m_allowedRange);
	bool isYClose = Tools::IsWithinRange(desired.YOffset, 0, m_allowedRange);
	
	if (isXClose && isYClose) {
		m_isFinished = true;
	}
	
	m_turret->TurnGivenOffset(desired);
}

bool AdjustTurretCommand::IsFinished() {
	return m_isFinished;
}

void AdjustTurretCommand::End() {
	// empty
}

void AdjustTurretCommand::Interrupted() {
	// empty
}



ManuallyControlTurretCommand::ManuallyControlTurretCommand(BaseFrisbeeTurret *turret, Axis *verticalAxis, Axis *rotateAxis) :
			SimpleCommand("ManuallyControlTurretCommand", false) {
	m_turret = turret;
	m_verticalAxis = verticalAxis;
	m_rotateAxis = rotateAxis;
	Requires(m_turret);
}

ManuallyControlTurretCommand::~ManuallyControlTurretCommand() {
	// empty
}

void ManuallyControlTurretCommand::Execute() {
	m_turret->TurnHorizontal(m_rotateAxis->Get());
	m_turret->TurnVertical(m_verticalAxis->Get());
}

