/**
 * vision.h
 * 
 * Classes all about using the camera, analyzing images, and sending
 * data back to the computer.
 * 
 * In progress, do not use.
 * 
 * Skyline Spartabots, Team 2976
 * Made for 2012 Robot Rumble
 */

#ifndef VISION_H_
#define VISION_H_

// Standard library
#include <utility>
#include <math.h>

// 3rd party libraries
#include "WPILib.h"
#include "Vision/HSLImage.h"

// Program modules
#include "component.h"


/**
 * The Target class is simply a collection of data
 * symbolizing each target.  At maximum, the robot should return a
 * vector containing only four Targets -- one for each backboard.
 */
struct Target
{
public:
	// The percieved width and height of the target
	double mWidth;
	double mHeight;
	
	// The score (the higher, the more likely it is a target)
	double mRawScore;
	double mScore;
	
	// The position of the target (in relation to the camera)
	double mPositionX; 
	double mPositionY; // Height
	double mPositionZ; // Depth
	
public:
	double GetMagnitude();
};

/**
 * The TargetFinder class is responsible for interfacing with the
 * camera, and doing any image analysis code.
 */
class TargetFinder : public BaseComponent
{
protected:
	
public:
	TargetFinder();
	void Run();
	vector<Target> GetTargets(HSLImage *);
protected:
	AxisCamera & GetCamera();
};

/**
 * The DashboardDataSender class is responsible for
 * getting data from a TargetFinder and sending it 
 * it over to the dashboard.
 */
class DashboardDataSender : public BaseComponent
{
public:
	DashboardDataSender();
};

#endif
