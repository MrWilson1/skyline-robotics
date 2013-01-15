/**
 * This file is the main entry point of the robot.
 * It loads up every existing profile, and lets you select which one to 
 * compile to.
 */

#include "Profiles/BlandRobotProfile.h"

#define BLAND 0
#define SOMEOTHERROBOT 1

// Modify these lines below:

#define MAINROBOT BLAND
// #define MAINROBOT SOMEOTHERROBOT

// Don't modify anymore.

#if MAINROBOT == BLAND
	START_ROBOT_CLASS(BlandRobotProfile);
#elif MAINROBOT == SOMEOTHERROBOT
	//START_ROBOT_CLASS(SomeOtherRobotProfile);
#endif
