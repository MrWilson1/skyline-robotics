#ifndef _TRACKSILVER_H
#define _TRACKSILVER_H

// Standard library
#include <utility>
#include <math.h>
#include <algorithm>

// 3rd party libraries
#include "WPILib.h"
#include "nivision.h"
#include "Vision/HSLImage.h"
#include "Vision/MonoImage.h"

// Program modules
#include "components.h"
#include "target.h"



struct Target{
	// Empty, meant to be a baseclass
};


class ImageTarget {
public:
	virtual Target ProcessImage(HSLImage *image);
};

class SilverImageTarget : ImageTarget {
public:
	SilverImageTarget();
	Target ProcessImage(HSLImage *image);
	struct Centroid : Target {
		int x;
		int y;
	};
	
protected:
	Threshold colorThreshold;
};


class MultithreadedTargetFinder{
public:
	MultithreadedTargetFinder(ImageTarget);
	void StartTask();
	void EndTask();
	Target *ReturnTargetData();

	
};

#endif
