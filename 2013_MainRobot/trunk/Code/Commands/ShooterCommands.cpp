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


ShooterCommand::AimTurret::AimTurret(
		FrisbeeAimer::Base *aimer, 
		FrisbeeTurret::Base *horizontalTurret,
		FrisbeeTurret::Base *verticalTurret,
		Tracking::TargetType desiredTarget,
		double allowedRange) :
		Command("AimTurret"),
		m_isFinished(false),
		m_desiredTarget(desiredTarget),
		m_allowedRange(allowedRange) {
	m_aimer = aimer;
	m_horizontalTurret = horizontalTurret;
	m_verticalTurret = verticalTurret;
	Requires(m_aimer);
	Requires(m_horizontalTurret);
	Requires(m_verticalTurret);
}


ShooterCommand::AimTurret::~AimTurret() {
	// empty
}

void ShooterCommand::AimTurret::Initialize() {
	// empty
}

void ShooterCommand::AimTurret::Execute() {
	// Stage 1: Gets the tracking information of the desired target.
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
		
	Tracking::Offset offset = target.ShooterOffset;
	
	// Stage 2: determines if the shooter is pointing at the target.
	bool isXDone = Tools::IsWithinRange(offset.XOffset, 0, m_allowedRange);
	bool isYDone = Tools::IsWithinRange(offset.YOffset, 0, m_allowedRange);
	
	// Stage 3: adjusts the turret as appropriate
	if (!isXDone) {
		if (offset.XOffset < 0) {
			m_horizontalTurret->SetSpeed(-0.3);
		} else if (offset.XOffset > 0){
			m_horizontalTurret->SetSpeed(0.3);
		}
	}
	if (!isYDone) {
		if (offset.YOffset < 0) {
			m_verticalTurret->SetSpeed(-0.3);
		} else if (offset.YOffset > 0){
			m_verticalTurret->SetSpeed(0.3);
		}
	}
	
	m_isFinished = isXDone and isYDone;
}

bool ShooterCommand::AimTurret::IsFinished() {
	return m_isFinished;
}

void ShooterCommand::AimTurret::End() {
	// empty
}

void ShooterCommand::AimTurret::Interrupted() {
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






/**
 * todo: Make two versions of this: one to manually go to some distance (replace Axis with doubles)
 * and another to control using Axis
 */
ShooterCommand::AdjustTurret::AdjustTurret(
		FrisbeeTurret::Base *horizontalTurret,
		FrisbeeTurret::Base *verticalTurret,
		double rotateSpeed,
		double verticalSpeed,
		double allowedRange) :
		SimpleCommand("AdjustTurretCommand", false),
		m_rotateSpeed(rotateSpeed),
		m_verticalSpeed(verticalSpeed) {
	m_horizontalTurret = horizontalTurret;
	m_verticalTurret = verticalTurret;
	Requires(m_horizontalTurret);
	Requires(m_verticalTurret);
}

ShooterCommand::AdjustTurret::~AdjustTurret() {
	// empty
}

void ShooterCommand::AdjustTurret::Execute() {
	m_horizontalTurret->SetSpeed(Tools::Limit(m_rotateSpeed, -1.0, 1.0));
	m_verticalTurret->SetSpeed(Tools::Limit(m_verticalSpeed, -1.0, 1.0));
}


ShooterCommand::ManuallyControlTurret::ManuallyControlTurret(FrisbeeTurret::Base *turretAxis, BaseAxis *inputAxis, const char *name) :
		SimpleCommand(name, false) {
	m_turret = turretAxis;
	m_axis = inputAxis;
	Requires(m_turret);
}

ShooterCommand::ManuallyControlTurret::~ManuallyControlTurret() {
	// empty
}

void ShooterCommand::ManuallyControlTurret::Execute() {
	m_turret->SetSpeed(m_axis->Get());
}

