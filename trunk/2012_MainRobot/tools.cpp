/**
 * tools.h
 * 
 * This file contains a variety of useful static functions to
 * perform common but useful tasks.
 * 
 * Skyline Spartabots, Team 2976
 * Made for 2012 Robot Rumble
 */

#include "tools.h"

double Tools::Coerce(double number, double rawMin, double rawMax, double adjustedMin, double adjustedMax)
{
	// Check inputs for validity.
	assert(rawMin <= rawMax);
	assert(adjustedMin <= adjustedMax);
	
	if (number < rawMin) {
		number = rawMin;
	} else if (number > rawMax) {
		number = rawMax;
	}
	
	double percentage = (number - rawMin) / (rawMax - rawMin);
	return percentage * (adjustedMax - adjustedMin) - 1; 
}
