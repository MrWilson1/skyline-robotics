#include "ShooterCommands.h"
#include "math.h"

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
		BaseAxisFrisbeeTurret *horizontalTurret,
		BaseAxisFrisbeeTurret *verticalTurret,
		Tracking::TargetType desiredTarget,
		double allowedRange) :
		Command("AimTurret"),
		m_isFinished(false),
		m_desiredTarget(desiredTarget),
		m_allowedRange(allowedRange),
		k_lowSpeed(0.3),
		k_mediumSpeed(0.6),
		k_highSpeed(0.9) {
	m_aimer = aimer;
	m_horizontalTurret = horizontalTurret;
	m_verticalTurret = verticalTurret;
	Requires(m_aimer);
	Requires(m_horizontalTurret);
	Requires(m_verticalTurret);
}

AimTurretCommand::AimTurretCommand(
		BaseFrisbeeAimer *aimer, 
		BaseAxisFrisbeeTurret *horizontalTurret, 
		BaseAxisFrisbeeTurret *verticalTurret,
		Tracking::TargetType desiredTarget, 
		double allowedRange,
		double lowSpeed,
		double mediumSpeed,
		double highSpeed) :
		Command("AimTurret"),
		m_isFinished(false),
		m_desiredTarget(desiredTarget),
		m_allowedRange(allowedRange),
		k_lowSpeed(lowSpeed),
		k_mediumSpeed(mediumSpeed),
		k_highSpeed(highSpeed) {
	m_aimer = aimer;
	m_horizontalTurret = horizontalTurret;
	m_verticalTurret = verticalTurret;
	Requires(m_aimer);
	Requires(m_horizontalTurret);
	Requires(m_verticalTurret);
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
		
	Tracking::Offset offset = target.ShooterOffset;
	double xDirection = 1, yDirection = 1;
	
	if (offset.XOffset < 0) {
		xDirection = -1;
	}
	
	if (offset.YOffset < 0) {
		yDirection = -1;
	}
	
	bool isXDone = Tools::IsWithinRange(offset.XOffset, 0, m_allowedRange);
	bool isYDone = Tools::IsWithinRange(offset.YOffset, 0, m_allowedRange);
	m_isFinished = isXDone and isYDone;
	
	if (!m_isFinished) {
		m_horizontalTurret->TurnGivenOffset(offset, xDirection, 57, 17, 8);
		m_verticalTurret->TurnGivenOffset(offset, yDirection, 33, 22, 11);
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
		BaseAxisFrisbeeTurret *horizontalTurret, 
		BaseAxisFrisbeeTurret *verticalTurret,
		BaseFrisbeeShooter *shooter) :
		CommandGroup("LoadAndFireCommand") {
	AddParallel(new FireFrisbeeCommand(shooter), 5.0);
	AddSequential(new LoadFrisbeeCommand(loader));
	AddSequential(new AimTurretCommand(aimer, horizontalTurret, verticalTurret, Tracking::ClosestOffset, 5));
}

LoadAndFireCommand::~LoadAndFireCommand() {
	// empty
}

/**
 * todo: Make two versions of this: one to manually go to some distance (replace Axis with doubles)
 * and another to control using Axis
 */
AdjustTurretCommand::AdjustTurretCommand(
		BaseAxisFrisbeeTurret *horizontalTurret,
		BaseAxisFrisbeeTurret *verticalTurret,
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

AdjustTurretCommand::~AdjustTurretCommand() {
	// empty
}

void AdjustTurretCommand::Execute() {
	m_horizontalTurret->SetMotor(Tools::Limit(m_rotateSpeed, -1.0, 1.0));
	m_verticalTurret->SetMotor(Tools::Limit(m_verticalSpeed, -1.0, 1.0));
}

ManuallyControlTurretCommand::ManuallyControlTurretCommand(BaseAxisFrisbeeTurret *horizontalTurret, BaseAxisFrisbeeTurret *verticalTurret, Axis *verticalAxis, Axis *rotateAxis) :
			SimpleCommand("ManuallyControlTurretCommand", false) {
	m_horizontalTurret = horizontalTurret;
	m_verticalTurret = verticalTurret;
	m_rotateAxis = rotateAxis;
	Requires(m_horizontalTurret);
	Requires(m_verticalTurret);
}

ManuallyControlTurretCommand::~ManuallyControlTurretCommand() {
	// empty
}

void ManuallyControlTurretCommand::Execute() {
	m_horizontalTurret->SetMotor(m_rotateAxis->Get());
	m_verticalTurret->SetMotor(m_verticalAxis->Get());
}

