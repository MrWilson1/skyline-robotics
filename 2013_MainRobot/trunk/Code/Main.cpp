/**
 * \mainpage 2013_MainRobot Documentation
 * 
 * \section intro_sec Introduction
 * 
 * This is the main codebase for Team 2976 for the 2013 game (Ultimate Ascent).
 * In case you're viewing this document from someplace else, you can find
 * the link to our code at [Google Code](http://code.google.com/p/skyline-robotics/source/browse/#svn%2F2013_MainRobot%253Fstate%253Dclosed)
 * 
 * A copy of our main can be downloaded from the repo using SVN:
 *  
 *     svn checkout http://skyline-robotics.googlecode.com/svn/2013_MainRobot/trunk local-folder-name
 * 
 * Our issue tracker can be found on [Trello](https://trello.com/board/4f8f9afb0bc9234208761bba)
 * 
 * \section about_architecture_sec About the Code
 * 
 * The codebase is split up into four different layers:
 * 
 * -   [Subsystems](\ref subsystems)
 *     -   Corresponds to physical components on the robot (arm, drive, etc)
 * -   Commands
 *     -   Corresponds to robot behavior. Commands use subsystems or other
 *         commands to script out complex behavior.
 * -   Operator Interface (OI)
 *     -   Maps input to commands, and handles all interactions from the driver.
 * -   Profile
 *     -   Each profile corresponds to a single robot. It specifies all the ports
 *         and precise parts used (Jaguars vs Talons, for example), all subsystems
 *         used, and the OI used.
 *         
 * Each layer is coded in a way to be interchangeable. For example, a 
 * 'ShootFrisbee' command requires some sort of Shooter subsystem, but 
 * can be passed in and use a SingleWheelShooter or SlingshotShooter (for 
 * example) without needing modification.
 */

/**
 * \file Main.cpp
 * 
 * \brief This file is the main entry point of the robot.
 * 
 * It loads up every existing profile, and lets you select which one to 
 * compile to.
 */

// Modify these lines below:

//#define ROBOT BLAND
//#define ROBOT TERMINATOR
//#define ROBOT MAINROBOT2013
#define ROBOT TESTBEDROBOT2013

// DO NOT MODIFY BELOW

#include "Profiles/BlandRobotProfile.h"
#include "Profiles/TerminatorRobotProfile.h"
#include "Profiles/MainRobot2013Profile.h"
#include "Profiles/TestbedRobot2013Profile.h"

#define BLAND 1
#define TERMINATOR 2
#define MAINROBOT2013 3
#define TESTBEDROBOT2013 4

#if ROBOT == BLAND
	START_ROBOT_CLASS(BlandRobotProfile);
#elif ROBOT == TERMINATOR
	START_ROBOT_CLASS(TerminatorRobotProfile);
#elif ROBOT == MAINROBOT2013
	START_ROBOT_CLASS(MainRobot2013Profile);
#elif ROBOT == TESTBEDROBOT2013
	START_ROBOT_CLASS(TestbedRobot2013Profile);
#endif
