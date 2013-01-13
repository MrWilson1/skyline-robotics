#include "Profiles/BlandRobotProfile.h"

#define BLAND 0
#define SOMEOTHERROBOT 1

#define MAINROBOT BLAND
// #define MAINROBOT SOMEOTHERROBOT

#if MAINROBOT == BLAND
	START_ROBOT_CLASS(BlandRobotProfile);
#elif MAINROBOT == SOMEOTHERROBOT
	//START_ROBOT_CLASS(SomeOtherRobotProfile);
#endif
