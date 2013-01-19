/**
 * This file is the main entry point of the robot.
 * It loads up every existing profile, and lets you select which one to 
 * compile to.
 */

// Modify these lines below:

#define MAINROBOT BLAND
// #define MAINROBOT SOMEOTHERROBOT


// DO NOT MODIFY BELOW

#include "Profiles/BlandRobotProfile.h"

#define BLAND 1
#define SOMEOTHERROBOT 2

#if MAINROBOT == BLAND
	START_ROBOT_CLASS(BlandRobotProfile);
#elif MAINROBOT == SOMEOTHERROBOT
	//START_ROBOT_CLASS(SomeOtherRobotProfile);
#endif
