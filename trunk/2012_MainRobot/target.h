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

#ifndef TARGET_H_
#define TARGET_H_

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


// Used for the corners of targets.

struct Coordinate {
public:
	Coordinate();
	void Set(float, float);
	float X;
	float Y;
};


/**
 * The Target class is simply a collection of data
 * symbolizing each target.  At maximum, the robot should return a
 * vector containing only four Targets -- one for each backboard.
 */
struct Target
{
public:
	// The percieved width and height of the target
	double Width;
	double Height;
	double Rotation;
	
	// The score (the higher, the more likely it is a target)
	double Score;
	
	// Corners
	Coordinate TopLeft;
	Coordinate TopRight;
	Coordinate BottomLeft;
	Coordinate BottomRight;
	
	// More data
	Coordinate Middle;
	double DistanceFromCamera;
	double XAngleFromCamera;
	double YAngleFromCamera;
};



namespace TargetUtils {
/*
static RectangleDescriptor rectangleDescriptor = {
		10,		// minWidth
		400,	// maxWidth
		10,		// minHeight
		400,	// maxHeight
};

static CurveOptions curveOptions = {
		IMAQ_NORMAL_IMAGE,	// Extraction mode
		1,					// Edge threshold
		IMAQ_NORMAL, 		// Filter size
		1,					// Min length
		7,					// Row step size
		6,					// Column step size
		23,					// Max end point gap
		0,					// Detect only closed curves?
		1					// Identify curves with subpixel accuracy?
};
static ShapeDetectionOptions shapeDetectionOptions = {
		IMAQ_GEOMETRIC_MATCH_ROTATION_INVARIANT,	// Detection mode
		NULL,				// Angle ranges (all)
		0,					// Num angle ranges
		{0, 100},			// Scale range
		700					// Minimum score
};
*/
static Threshold threshold = Threshold(
		243,				// Red min
		255,				// Red max
		141,				// Green min
		255,				// Green max
		161,				// Blue min
		255					// Blue max
);

/**
 * Ok, so WPILib provided 'BinaryImage', which helps with
 * image manipulation.  Unfortunately, it was only able
 * to find ellipses, not rectangles.
 * 
 * I inherited BinaryImage and effectively wrote my
 * own version to include rectangle handling.
 */
class SaneBinaryImage : public BinaryImage
{
public:
	SaneBinaryImage();
	virtual ~SaneBinaryImage();
	
	vector<RectangleMatch> *DetectRectangles(
			RectangleDescriptor *,
			CurveOptions *,
			ShapeDetectionOptions *,
			ROI *);

	vector<RectangleMatch> *DetectRectangles(
			RectangleDescriptor *);
};

} // End namespace.






/**
 * The TargetFinder class is responsible for interfacing with the
 * camera, and doing any image analysis code.
 * 
 * Note: the image processing settings was actually tested and
 * debugged using the NI Vision Assistant tool, then ported
 * over to code.  See the 2010 and 2012 vision samples for 
 * templates.
 * 
 * See also the 
 */
class TargetFinder : public BaseController
{
public:
	TargetFinder();
	void Run();
	vector<Target> GetTargets(HSLImage *);
protected:
	AxisCamera & GetCamera();
	Timer *mTimer;
	static const double kTimerPeriod = 0.5;	// In seconds
	double CalculateDistanceBasedOnWidth(double);
	static const double kTargetWidthInches = 24;
	static const double kTargetHeightInches = 18;
};
/*
class TargetFinderThread : public Task
{
protected:
	const char *mName;
	const char *mCameraIp;
	vector<Target> *mVectorTarget;
	vector<Target> *mVectorTargetCached;
	static const double kTargetWidthInches = 24;
	static const double kTargetHeightInches = 18;
	
public:
	TargetFinderThread(
			const char *,
			const char *,
			vector<Target> *);
	~TargetFinderThread();
	
protected:
	static void TaskWrapper(void *);
	virtual void Run();
	vector<Target> *GetTargets(HSLImage *);
	AxisCamera & GetCamera(void);
	double CalculateDistanceBasedOnWidth(double);
};

*/
#endif
