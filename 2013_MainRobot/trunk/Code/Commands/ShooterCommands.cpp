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

FireFrisbeeAutoDistanceCommand::FireFrisbeeAutoDistanceCommand(BaseFrisbeeShooter *shooter) :
		Command("FireFrisbee") {
	m_shooter = shooter;
	Requires(m_shooter);
}

FireFrisbeeAutoDistanceCommand::~FireFrisbeeAutoDistanceCommand() {
	// empty
}

void FireFrisbeeAutoDistanceCommand::Initialize() {
	// empty
}

void FireFrisbeeAutoDistanceCommand::Execute() {
	m_shooter->ShootFrisbee();
}

bool FireFrisbeeAutoDistanceCommand::IsFinished() {
	return true;
}

void FireFrisbeeAutoDistanceCommand::End() {
	// empty
}

void FireFrisbeeAutoDistanceCommand::Interrupted() {
	// empty
}

FireFrisbeeManualDistanceCommand::FireFrisbeeManualDistanceCommand(BaseFrisbeeShooter *shooter, float setDistanceInInches) :
		Command("FireFrisbee") {
	m_shooter = shooter;
	distanceInInches = setDistanceInInches;
	Requires(m_shooter);
}

FireFrisbeeManualDistanceCommand::~FireFrisbeeManualDistanceCommand() {
	// empty
}

void FireFrisbeeManualDistanceCommand::Initialize() {
	// empty
}

void FireFrisbeeManualDistanceCommand::Execute() {
	m_shooter->ShootFrisbee(distanceInInches);
}

bool FireFrisbeeManualDistanceCommand::IsFinished() {
	return true;
}

void FireFrisbeeManualDistanceCommand::End() {
	// empty
}

void FireFrisbeeManualDistanceCommand::Interrupted() {
	// empty
}

EjectFrisbeeCommand::EjectFrisbeeCommand(BaseFrisbeeShooter *shooter) :
		Command("FireFrisbee") {
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
