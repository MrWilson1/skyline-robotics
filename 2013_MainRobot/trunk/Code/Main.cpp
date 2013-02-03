/**
 * This file is the main entry point of the robot.
 * It loads up every existing profile, and lets you select which one to 
 * compile to.
 */

// Modify these lines below:

#define ROBOT BLAND
//#define ROBOT TERMINATOR
//#define ROBOT MAINROBOT2013


// DO NOT MODIFY BELOW

#include "Profiles/BlandRobotProfile.h"
#include "Profiles/TerminatorRobotProfile.h"
#include "Profiles/MainRobot2013Profile.h"

#define BLAND 1
#define TERMINATOR 2
#define MAINROBOT2013 3

#if ROBOT == BLAND
	START_ROBOT_CLASS(BlandRobotProfile);
#elif ROBOT == TERMINATOR
	START_ROBOT_CLASS(TerminatorRobotProfile);
#elif ROBOT == MAINROBOT2013
	START_ROBOT_CLASS(MainRobot2013Profile);
#endif
