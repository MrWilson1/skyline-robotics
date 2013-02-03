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

AimTurretCommand::AimTurretCommand(BaseFrisbeeAimer *aimer, BaseFrisbeeTurret *turret, float allowedRange) :
		Command("AimTurret"),
		m_isFinished(false),
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
	Tracking::Target target = m_aimer->GetClosestTargetByOffset();
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
		Command("FireFrisbee"),
		m_distanceInInches(1.0) {
	m_shooter = shooter;
	Requires(m_shooter);
}

FireFrisbeeCommand::FireFrisbeeCommand(BaseFrisbeeShooter *shooter, double distanceInInches) :
		Command("FireFrisbee") {
	m_shooter = shooter;
	m_distanceInInches = distanceInInches;
	Requires(m_shooter);
}

FireFrisbeeCommand::~FireFrisbeeCommand() {
	// empty
}

void FireFrisbeeCommand::Initialize() {
	// empty
}

void FireFrisbeeCommand::Execute() {
	m_shooter->ShootFrisbee(m_distanceInInches);
}

bool FireFrisbeeCommand::IsFinished() {
	return true;
}

void FireFrisbeeCommand::End() {
	// empty
}

void FireFrisbeeCommand::Interrupted() {
	// empty
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
	AddSequential(new AimTurretCommand(aimer, turret, 5));
	AddSequential(new FireFrisbeeCommand(shooter));
}

LoadAndFireCommand::~LoadAndFireCommand() {
	// empty
}