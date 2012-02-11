/**
 * vision.h
 * 
 * Classes all about using the camera, analyzing images, and sending
 * data back to the computer.
 * 
 * In progress, do not use.
 * 
 * TODO: Write this.
 * 
 * Skyline Spartabots, Team 2976
 * Made for 2012 Robot Rumble
 */

#include "vision.h"

double Target::GetMagnitude()
{
	double xSquared = mPositionX * mPositionX;
	double ySquared = mPositionY * mPositionY;
	double zSquared = mPositionZ * mPositionZ;
	return sqrt(xSquared + ySquared + zSquared);
}


TargetFinder::TargetFinder()
{
	
	
}

void TargetFinder::Run()
{
	AxisCamera &camera = GetCamera();
	HSLImage *image = camera.GetImage();	// Get image.
	//vector<Target> targets = GetTargets(image);
	
}
/*
vector<Target> TargetFinder::GetTargets(HSLImage *image)
{
	
	
	
}
*/
AxisCamera & TargetFinder::GetCamera()
{
	AxisCamera &camera = AxisCamera::GetInstance("10.29.76.11");
	camera.WriteResolution(AxisCamera::kResolution_320x240);
	camera.WriteCompression(20);
	camera.WriteBrightness(0);
	return camera;
}


