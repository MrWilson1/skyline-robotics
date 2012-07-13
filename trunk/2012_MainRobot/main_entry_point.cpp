#include "MainRobot.h"
#include "PrototypeRobot.h"

#define PROTOTYPE

#ifndef PROTOTYPE
START_ROBOT_CLASS(MainRobot);
#endif

#ifdef PROTOTYPE
START_ROBOT_CLASS(PrototypeRobot);
#endif

