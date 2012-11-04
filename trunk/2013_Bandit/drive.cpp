#include "drive.h"

Drive::Drive(RobotDrive *robotDrive) {
	mRobotDrive = robotDrive;
}

RobotDrive* Drive::GetDrive() {
	return mRobotDrive;
}
