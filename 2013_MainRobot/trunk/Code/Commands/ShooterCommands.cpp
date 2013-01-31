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

AimTurretCommand::AimTurretCommand(BaseFrisbeeAimer *aimer) :
		Command("AimTurret") {
	m_aimer = aimer;
	Requires(m_aimer);
}

AimTurretCommand::~AimTurretCommand() {
	// empty
}

void AimTurretCommand::Initialize() {
	// empty
}

void AimTurretCommand::Execute() {
	// empty
}

bool AimTurretCommand::IsFinished() {
	return true;
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

LoadAndFire::LoadAndFire(BaseFrisbeeLoader *loader, BaseFrisbeeAimer *aimer, BaseFrisbeeShooter *shooter) :
	CommandGroup("LoadAndFire") {
	AddSequential(new LoadFrisbeeCommand(loader));
	AddSequential(new AimTurretCommand(aimer));
	AddSequential(new FireFrisbeeCommand(shooter));
}

LoadAndFire::~LoadAndFire() {
	// empty
}
