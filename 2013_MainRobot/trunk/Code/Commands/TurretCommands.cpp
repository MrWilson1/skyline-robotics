#include "TurretCommands.h"

TurretCommand::AimTurret::AimTurret(
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


TurretCommand::AimTurret::~AimTurret() {
	// empty
}

void TurretCommand::AimTurret::Initialize() {
	// empty
}

void TurretCommand::AimTurret::Execute() {
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
	
	SmartDashboard::PutNumber("Aimer x offset", target.ShooterOffset.XOffset);
	SmartDashboard::PutNumber("Aimer x offset", target.ShooterOffset.YOffset);
	SmartDashboard::PutNumber("Aimer distance", target.DistanceInInches / 12.0);
	
	return;
	
	if (target.DistanceInInches == 0) {
		// The Vision code sets the distance to 0 if the target type isn't found.
		return;
	}
		
	Tracking::Offset offset = target.ShooterOffset;
	
	float xCenter = 240;
	float yCenter = 180;
	
	// Stage 2: determines if the shooter is pointing at the target.
	bool isXDone = Tools::IsWithinRange(offset.XOffset, xCenter, m_allowedRange);
	bool isYDone = Tools::IsWithinRange(offset.YOffset, yCenter, m_allowedRange);
	
	// Stage 3: adjusts the turret as appropriate
	if (!isXDone) {
		if (offset.XOffset < xCenter) {
			SmartDashboard::PutString("Turret XOffset", "neg");
			m_horizontalTurret->SetSpeed(-1.0);
		} else if (offset.XOffset > xCenter){
			m_horizontalTurret->SetSpeed(1.0);
			SmartDashboard::PutString("Turret XOffset", "pos");
		} else {
			SmartDashboard::PutString("Turret XOffset", "none");
		}
	}
	if (!isYDone) {
		if (offset.YOffset < yCenter) {
			m_verticalTurret->SetSpeed(-0.75);
			SmartDashboard::PutString("Turret YOffset", "neg");
		} else if (offset.YOffset > yCenter){
			m_verticalTurret->SetSpeed(0.75);
			SmartDashboard::PutString("Turret YOffset", "pos");
		} else {
			SmartDashboard::PutString("Turret YOffset", "none");
		}
		
	}
	
	m_isFinished = isXDone and isYDone;
}

bool TurretCommand::AimTurret::IsFinished() {
	return m_isFinished;
}

void TurretCommand::AimTurret::End() {
	// empty
}

void TurretCommand::AimTurret::Interrupted() {
	// empty
}










/**
 * todo: Make two versions of this: one to manually go to some distance (replace Axis with doubles)
 * and another to control using Axis
 */
TurretCommand::AdjustTurret::AdjustTurret(
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

TurretCommand::AdjustTurret::~AdjustTurret() {
	// empty
}

void TurretCommand::AdjustTurret::Execute() {
	m_horizontalTurret->SetSpeed(Tools::Limit(m_rotateSpeed, -1.0, 1.0));
	m_verticalTurret->SetSpeed(Tools::Limit(m_verticalSpeed, -1.0, 1.0));
}



/**
 * \brief Manually adjusts where the turret is pointing by plugging in raw axis speeds
 */
TurretCommand::AdjustSingleTurret::AdjustSingleTurret(
		FrisbeeTurret::Base *turret,
		double speed) :
		SimpleCommand("TurretCommand::AdjustSingleTurret", true),
		m_speed(speed){
	m_turret = turret;
	Requires(m_turret);
}

TurretCommand::AdjustSingleTurret::~AdjustSingleTurret() {
	// empty
}

void TurretCommand::AdjustSingleTurret::Execute() {
	m_turret->SetSpeed(m_speed);
}
	








TurretCommand::AdjustTurretAngle::AdjustTurretAngle(
		FrisbeeTurret::Base *turret,
		TurretPosition::Base *position,
		float angle) :
		SimpleCommand("TurretCommand::AdjustTurretAngle", false),
		m_angle(angle)  {
	m_turret = turret;
	m_position = position;
	Requires(m_turret);
	Requires(m_position);
}

TurretCommand::AdjustTurretAngle::~AdjustTurretAngle() {
	// empty
}
	
void TurretCommand::AdjustTurretAngle::Execute() {
	float current = m_position->GetAngle();
	if (Tools::IsWithinRange(m_angle, current, 4)) {
		return;
	} else if (m_angle > (current + 4)) {
		m_turret->SetSpeed(0.75);
	} else if (m_angle < (current - 4)) {
		m_turret->SetSpeed(-0.75);
	} else {
		return;
	}
}

bool TurretCommand::AdjustTurretAngle::IsFinished() {
	return Tools::IsWithinRange(m_angle, m_position->GetAngle(), 4);
}










TurretCommand::ManuallyControlTurret::ManuallyControlTurret(FrisbeeTurret::Base *turretAxis, BaseAxis *inputAxis, const char *name) :
		SimpleCommand(name, false) {
	m_turret = turretAxis;
	m_axis = inputAxis;
	Requires(m_turret);
}

TurretCommand::ManuallyControlTurret::~ManuallyControlTurret() {
	// empty
}

void TurretCommand::ManuallyControlTurret::Execute() {
	m_turret->SetSpeed(m_axis->Get());
}





TurretCommand::MoveTurretHome::MoveTurretHome(
		FrisbeeTurret::Base *turretAxis,
		TurretPosition::Base *position,
		const char *name) :
		SimpleCommand(name, false) {
	m_turret = turretAxis;
	m_position = position;
	Requires(m_turret);
	Requires(m_position);
}

TurretCommand::MoveTurretHome::~MoveTurretHome() {
	// empty
}

void TurretCommand::MoveTurretHome::Execute() {
	TurretPosition::Position position = m_position->GetPosition();
	switch(position) {
	case (TurretPosition::kCenter):
		m_turret->SetSpeed(0);
		break;
	case (TurretPosition::kRight):
		m_turret->SetSpeed(-0.75);
		break;
	case (TurretPosition::kLeft):
		m_turret->SetSpeed(0.75);
		break;
	default:
		m_turret->SetSpeed(0);
	}
}

bool TurretCommand::MoveTurretHome::IsFinished() {
	TurretPosition::Position position = m_position->GetPosition();
	return (position == TurretPosition::kCenter) || (position == TurretPosition::kError);
}




TurretCommand::MoveBothTurretsHome::MoveBothTurretsHome(
			FrisbeeTurret::Base *horizTurret,
			TurretPosition::Base *horizPosition,
			FrisbeeTurret::Base *verticalTurret,
			TurretPosition::Base *verticalPosition) :
			CommandGroup("TurretCommand::MoveBothTurretsHome") {
	AddParallel(new TurretCommand::MoveTurretHome(
			horizTurret, 
			horizPosition, 
			"TurretCommand::MoveTurretHome Horizontal"));
	AddParallel(new TurretCommand::MoveTurretHome(
			verticalTurret, 
			verticalPosition, 
			"TurretCommand::MoveTurretHome Vertical"));
}

TurretCommand::MoveBothTurretsHome::~MoveBothTurretsHome() {
	// empty
}






TurretCommand::SetTurretHome::SetTurretHome(
		TurretPosition::Base *position,
		const char *name) :
		SimpleCommand(name, true)  {
	m_position = position;
}

TurretCommand::SetTurretHome::~SetTurretHome() {
	//empty
}

void TurretCommand::SetTurretHome::Execute() {
	m_position->SetHome();
}



TurretCommand::SetBothTurretsHome::SetBothTurretsHome(
			TurretPosition::Base *horizPosition,
			TurretPosition::Base *verticalPosition) :
			CommandGroup("TurretCommand::SetBothTurretsHome") {
	AddParallel(new TurretCommand::SetTurretHome(
			horizPosition, 
			"TurretCommand::SetTurretHome Horizontal"));
	AddParallel(new TurretCommand::SetTurretHome(
			verticalPosition, 
			"TurretCommand::SetTurretHome Vertical"));
}

TurretCommand::SetBothTurretsHome::~SetBothTurretsHome() {
	// empty
}



