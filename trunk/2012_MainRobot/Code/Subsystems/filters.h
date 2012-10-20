#ifndef _FILTERS_H
#define _FILTERS_H

// System libraries
#include <utility.h>
#include <math.h>
#include <algorithm>

#include "../Definitions/Components.h"
#include "../tools.h"

struct DriveSpeed
{
	DriveSpeed();
	DriveSpeed(float, float);
	float Left;
	float Right;
};

namespace Filter
{
	DriveSpeed SquareInput(DriveSpeed);
	DriveSpeed ReverseDirection(DriveSpeed);
	
	DriveSpeed AddSpeedFactor(DriveSpeed, float);
	DriveSpeed Straighten(DriveSpeed);
	DriveSpeed AddTruncation(DriveSpeed);
	
	float truncate(float);

}

#endif
