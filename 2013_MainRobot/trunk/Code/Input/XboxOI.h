#ifndef XBOX_OI_H
#define XBOX_OI_H

#include "WPILib.h"

#include "Xbox.h"
#include "BaseOI.h"
#include "../Commands/DriveCommands.h"
#include "../Subsystems/DriveSubsystem.h"

class XboxOI : public BaseOI
{
public:
	XboxOI(XboxController *xbox, BaseDrive *drive);
	virtual ~XboxOI();
	
	TankValues GetTankValues();
	ArcadeValues GetArcadeValues();
	
	void SetupTeleop();
	
private:
	XboxController *m_xbox;
	BaseDrive *m_drive;
	
	JoystickButton *m_spasmButton;
	Command *m_teleopCommand;
};

#endif
