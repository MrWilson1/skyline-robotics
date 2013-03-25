#include "ShooterCommands.h"
#include "math.h"

ShooterCommand::LoadFrisbee::LoadFrisbee(FrisbeeLoader::Base *loader) :
		Command("LoadFrisbee"),
		m_isFinished(false) {
	m_loader = loader;
	Requires(m_loader);
}

ShooterCommand::LoadFrisbee::~LoadFrisbee() {
	// empty
}

void ShooterCommand::LoadFrisbee::Initialize() {
	// empty
}

void ShooterCommand::LoadFrisbee::Execute() {
	/*if (m_loader->IsFrisbeePrepared()) {
		m_loader->LoadFrisbee();
		m_isFinished = true;
		SmartDashboard::PutString("LoadFrisbeeDebug", "load");
	} else {
		m_loader->PrepareFrisbee();
		SmartDashboard::PutString("LoadFrisbeeDebug", "prepare");
	}*/
	m_loader->LoadFrisbee();
	m_isFinished = true;
}

bool ShooterCommand::LoadFrisbee::IsFinished() {
	return m_isFinished;
}

void ShooterCommand::LoadFrisbee::End() {
	// empty
}

void ShooterCommand::LoadFrisbee::Interrupted() {
	// empty
}






ShooterCommand::FireFrisbee::FireFrisbee(FrisbeeShooter::Base *shooter) :
		Command("FireFrisbee"),
		m_speed(1.0) {
	m_shooter = shooter;
	Requires(m_shooter);
}

ShooterCommand::FireFrisbee::FireFrisbee(FrisbeeShooter::Base *shooter, float speed) :
		Command("FireFrisbee"),
		m_speed(speed) {
	m_shooter = shooter;
	Requires(m_shooter);
}

ShooterCommand::FireFrisbee::~FireFrisbee() {
	// empty
}

void ShooterCommand::FireFrisbee::Initialize() {
	// empty
}

void ShooterCommand::FireFrisbee::Execute() {
	m_shooter->SetFrisbeeSpeed(m_speed);
}

bool ShooterCommand::FireFrisbee::IsFinished() {
	return false;
}

void ShooterCommand::FireFrisbee::End() {
	m_shooter->StopFrisbee();
}

void ShooterCommand::FireFrisbee::Interrupted() {
	m_shooter->StopFrisbee();
}



ShooterCommand::FireFrisbeeWithAdjustableSpeed::FireFrisbeeWithAdjustableSpeed(
		FrisbeeShooter::Base *shooter,
		BaseAxis *speedAxis) :
		SimpleCommand("ShooterCommand::FireFrisbeeWithAdjustableSpeed", false) {
	m_shooter = shooter;
	m_speedAxis = speedAxis;
	
	Requires(m_shooter);
}

ShooterCommand::FireFrisbeeWithAdjustableSpeed::~FireFrisbeeWithAdjustableSpeed() {
	// empty
}

void ShooterCommand::FireFrisbeeWithAdjustableSpeed::Execute() {
	float speed = m_speedAxis->Get();
	m_shooter->SetFrisbeeSpeed(speed);
}

void ShooterCommand::FireFrisbeeWithAdjustableSpeed::End() {
	m_shooter->SetFrisbeeSpeed(0);
}

void ShooterCommand::FireFrisbeeWithAdjustableSpeed::Interrupted() {
	m_shooter->SetFrisbeeSpeed(0);
}


ShooterCommand::SmartDashboardFireFrisbee::SmartDashboardFireFrisbee(FrisbeeShooter::Base *shooter) :
		SimpleCommand("ShooterCommand::SmartDashboardFireFrisbee", false),
		m_conversionFactor(1.0){
	m_shooter = shooter;
	SmartDashboard::PutNumber("ShooterSpeed", 0.0);
}

ShooterCommand::SmartDashboardFireFrisbee::SmartDashboardFireFrisbee(FrisbeeShooter::Base *shooter, double conversionFactor) :
		SimpleCommand("ShooterCommand::SmartDashboardFireFrisbee", false),
		m_conversionFactor(conversionFactor) {
	m_shooter = shooter;
	SmartDashboard::PutNumber("ShooterSpeed", 0.0);
	SmartDashboard::PutNumber("ShooterConversionFactor", m_conversionFactor);
}

ShooterCommand::SmartDashboardFireFrisbee::~SmartDashboardFireFrisbee() {
	// empty
}

void ShooterCommand::SmartDashboardFireFrisbee::Execute() {
	float speed = SmartDashboard::GetNumber("ShooterSpeed");
	m_conversionFactor = SmartDashboard::GetNumber("ShooterConversionFactor");
	m_shooter->SetFrisbeeSpeed(speed / m_conversionFactor);
}


ShooterCommand::LoadAndFire::LoadAndFire(
		FrisbeeLoader::Base *loader,
		FrisbeeShooter::Base *shooter) :
		CommandGroup("LoadAndFireCommand") {
	AddParallel(new ShooterCommand::FireFrisbee(shooter), 5.0);
	AddSequential(new ShooterCommand::LoadFrisbee(loader));
}

ShooterCommand::LoadAndFire::~LoadAndFire() {
	// empty
}


