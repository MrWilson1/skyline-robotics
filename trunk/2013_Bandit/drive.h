#ifndef DRIVE_H_
#define DRIVE_H_

#include "component.h"
#include "WPILib.h"

class Drive : public Component {
public:
	Drive(RobotDrive *);
	RobotDrive *GetDrive();
protected:
	RobotDrive *mRobotDrive;
};

#endif
