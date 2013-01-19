#include "ShooterCommands.h"

LoadFrisbeeCommand::LoadFrisbeeCommand(BaseFrisbeeLoader *loader) :
		Command("LoadFrisbee") {
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
	m_loader->PrepareFrisbee();
	if (m_loader->IsFrisbeePrepared()) {
		m_loader->LoadFrisbee();
	}
}

bool LoadFrisbeeCommand::IsFinished() {
	return true;
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
	
}

void AimTurretCommand::End() {
	// empty
}

void AimTurretCommand::Interrupted() {
	// empty
}

FireFrisbeeMaxSpeedCommand::FireFrisbeeMaxSpeedCommand(BaseFrisbeeShooter *shooter) :
		Command("FireFrisbee") {
	m_shooter = shooter;
	Requires(m_shooter);
}

FireFrisbeeMaxSpeedCommand::~FireFrisbeeMaxSpeedCommand() {
	// empty
}

void FireFrisbeeMaxSpeedCommand::Initialize() {
	// empty
}

void FireFrisbeeMaxSpeedCommand::Execute() {
	m_shooter->ShootFrisbee();
}

bool FireFrisbeeMaxSpeedCommand::IsFinished() {
	return true;
}

void FireFrisbeeMaxSpeedCommand::End() {
	// empty
}

void FireFrisbeeMaxSpeedCommand::Interrupted() {
	// empty
}

FireFrisbeeSetSpeedCommand::FireFrisbeeSetSpeedCommand(BaseFrisbeeShooter *shooter, float setSpeed) :
		Command("FireFrisbee") {
	m_shooter = shooter;
	speed = setSpeed;
	Requires(m_shooter);
}

FireFrisbeeSetSpeedCommand::~FireFrisbeeSetSpeedCommand() {
	// empty
}

void FireFrisbeeSetSpeedCommand::Initialize() {
	// empty
}

void FireFrisbeeSetSpeedCommand::Execute() {
	m_shooter->ShootFrisbee(speed);
}

bool FireFrisbeeSetSpeedCommand::IsFinished() {
	return true;
}

void FireFrisbeeSetSpeedCommand::End() {
	// empty
}

void FireFrisbeeSetSpeedCommand::Interrupted() {
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
