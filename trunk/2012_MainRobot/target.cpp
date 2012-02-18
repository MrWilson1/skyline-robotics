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

// 3rd party modules
#include "target.h"
#include "Vision/ImageBase.h"



Coordinate::Coordinate()
{
	Coordinate::X = 0;
	Coordinate::Y = 0;
}

void Coordinate::Set(float x, float y)
{
	Coordinate::X = x;
	Coordinate::Y = y;
}

TargetUtils::SaneBinaryImage::SaneBinaryImage(void) : BinaryImage()
{
	// Empty
}

TargetUtils::SaneBinaryImage::~SaneBinaryImage(void)
{
	// Empty
}

vector<RectangleMatch> *TargetUtils::SaneBinaryImage::DetectRectangles(
		RectangleDescriptor *rectangleDescriptor,
		CurveOptions *curveOptions,
		ShapeDetectionOptions *shapeDetectionOptions,
		ROI *roi)
{
	int numberOfMatches;
	RectangleMatch *rectangleMatch = imaqDetectRectangles(
			m_imaqImage,			// Inherited member value
			rectangleDescriptor,
			curveOptions,
			shapeDetectionOptions,
			roi,
			&numberOfMatches		// Is modified by function.
	);
	
	vector<RectangleMatch> *rectangles = new vector<RectangleMatch>;
	
	if (rectangleMatch = NULL) {
		return rectangles;
	}
	for (int i = 0; i < numberOfMatches; i++) {
		rectangles->push_back(rectangleMatch[i]);
	}
	imaqDispose(rectangleMatch);
	return rectangles;
}

vector<RectangleMatch> *TargetUtils::SaneBinaryImage::DetectRectangles(
		RectangleDescriptor *rectangleDescriptor)
{
	vector<RectangleMatch> *rectangles = DetectRectangles(
			rectangleDescriptor,
			NULL,
			NULL,
			NULL);
	return rectangles;
}





TargetFinder::TargetFinder()
{
	mTimer = new Timer();
	mTimer->Start();
}


void TargetFinder::Run()
{
	SmartDashboard::GetInstance()->Log(mTimer->Get(), "Camera timer");
	if (mTimer->HasPeriodPassed(kTimerPeriod)) {
		SmartDashboard::GetInstance()->Log(mTimer->Get(), "Camera timer prev");
		AxisCamera &camera = GetCamera();
		HSLImage *image = camera.GetImage();	// Get image.
		vector<Target> targets = GetTargets(image);
		delete image;
		int size = (int) targets.size();
		if (size > 0) {
			// Just the first target
			Target t = targets.at(0);
			
			SmartDashboard::GetInstance()->Log(t.Width, "t.Width");
			SmartDashboard::GetInstance()->Log(t.Height, "t.Height");
			SmartDashboard::GetInstance()->Log(t.Rotation, "t.Rotation");
			
			SmartDashboard::GetInstance()->Log(t.Score, "t.Score");
			
			SmartDashboard::GetInstance()->Log(t.TopLeft.X, "t.TopLeft.x");
			SmartDashboard::GetInstance()->Log(t.TopLeft.Y, "t.TopLeft.y");
			SmartDashboard::GetInstance()->Log(t.TopRight.X, "t.TopRight.x");
			SmartDashboard::GetInstance()->Log(t.TopRight.Y, "t.TopRight.y");
			SmartDashboard::GetInstance()->Log(t.BottomLeft.X, "t.BottomLeft.x");
			SmartDashboard::GetInstance()->Log(t.BottomLeft.Y, "t.BottomLeft.y");
			SmartDashboard::GetInstance()->Log(t.BottomRight.X, "t.BottomRight.x");
			SmartDashboard::GetInstance()->Log(t.BottomRight.Y, "t.BottomRight.y");
			
			SmartDashboard::GetInstance()->Log(t.Middle.X, "t.Middle.x");
			SmartDashboard::GetInstance()->Log(t.Middle.Y, "t.Middle.y");
			SmartDashboard::GetInstance()->Log(t.DistanceFromCamera, "t.DistanceFromCamera");
		}
	}
}

namespace TargetUtils
{
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

}

vector<Target> TargetFinder::GetTargets(HSLImage *image)
{
	SmartDashboard::GetInstance()->Log("Grabbing Picture", "TargetFinder::GetTargets");
	BinaryImage *thresholdImage = image->ThresholdRGB(TargetUtils::threshold);    // Get only colors within range
	BinaryImage *bigObjectsImage = thresholdImage->RemoveSmallObjects(false, 1);  // Remove small objects
	BinaryImage *convexHullImage = bigObjectsImage->ConvexHull(false);  		  // Rill in partial and full rectangles
	TargetUtils::SaneBinaryImage *preRectangleImage = (TargetUtils::SaneBinaryImage *) convexHullImage;
	vector<RectangleMatch> *rectangles = preRectangleImage->DetectRectangles(
			&TargetUtils::rectangleDescriptor,
			&TargetUtils::curveOptions,
			&TargetUtils::shapeDetectionOptions,
			NULL
	);

	delete thresholdImage;
	delete bigObjectsImage;
	delete convexHullImage;
	delete preRectangleImage;

	vector<Target> targets;
	
	int size = (int) rectangles->size();
	SmartDashboard::GetInstance()->Log(size, "Number of targets");
	
	if (size == 0) {
		SmartDashboard::GetInstance()->Log("None found", "Camera Pics");
		delete rectangles;
		return targets;		// Empty vector
	}
	SmartDashboard::GetInstance()->Log("Found", "Camera Pics");
	
	for (int i=0; i<size; i++) {
		Target t;
		RectangleMatch r = rectangles->at(i);
		
		t.Width = r.width;
		t.Height = r.height;
		t.Rotation = r.rotation;	// Rotation from camera's horizontal axis.
		
		t.Score = r.score;
		t.TopLeft.Set(r.corner[0].x, r.corner[0].y);
		t.TopRight.Set(r.corner[1].x, r.corner[1].y);
		t.BottomRight.Set(r.corner[2].x, r.corner[2].y);
		t.BottomLeft.Set(r.corner[3].x, r.corner[3].y);
		
		t.DistanceFromCamera = CalculateDistanceBasedOnWidth(t.Width);
		
		
	}
	
	delete rectangles;
	return targets;
}

/**
 * Input:
 *   - Width (in pixels)
 * 
 * Output:
 *   - Distance from the camera to approx the center of the target
 *     (in inches)
 */
double TargetFinder::CalculateDistanceBasedOnWidth(double widthInPixels)
{
	// Based on exeriments we conducted...
	// Axis 206 Network camera
	// 640x480
	// The dial contains a focus -- the groove over the 'ar' in
	// 'Near' should be just a hair to the left of the bump.
	
	double distance = (17490 / widthInPixels) - 6.97; 
	return distance;
}

AxisCamera & TargetFinder::GetCamera()
{
	AxisCamera &camera = AxisCamera::GetInstance("10.29.76.11");
	camera.WriteResolution(AxisCamera::kResolution_320x240);
	camera.WriteCompression(100);
	camera.WriteBrightness(0);
	return camera;
}





TargetFinderThread::TargetFinderThread(
			const char *threadName,
			const char *cameraIp,
			vector<Target> *vectorTarget) :
	Task("TargetFinderThread", (FUNCPTR)TargetFinderThread::TaskWrapper)
{
	mName = threadName;
	mCameraIp = cameraIp;
	mVectorTarget = vectorTarget;
	
	Task::Start((UINT32)this);
}

void TargetFinderThread::TaskWrapper(void *ThisObject)
{
	// The task can only run C-style functions (ie static methods of
	// classes.  This function is a static function, but points the
	// actual object to its appropiate 'Run' method.
	TargetFinderThread *self = (TargetFinderThread *) ThisObject;
	
	self->Run();
}

void TargetFinderThread::Run(void)
{
	AxisCamera &camera = GetCamera();
	HSLImage *image = camera.GetImage();	// Get image.
	vector<Target> *targets = GetTargets(image);
	delete image;
	
	
	
	
}

vector<Target> *TargetFinderThread::GetTargets(HSLImage *image)
{
	BinaryImage *thresholdImage = image->ThresholdRGB(TargetUtils::threshold);    // Get only colors within range
	BinaryImage *bigObjectsImage = thresholdImage->RemoveSmallObjects(false, 1);  // Remove small objects
	BinaryImage *convexHullImage = bigObjectsImage->ConvexHull(false);  		  // Rill in partial and full rectangles
	TargetUtils::SaneBinaryImage *preRectangleImage = (TargetUtils::SaneBinaryImage *) convexHullImage;
	vector<RectangleMatch> *rectangles = preRectangleImage->DetectRectangles(
			&TargetUtils::rectangleDescriptor,
			&TargetUtils::curveOptions,
			&TargetUtils::shapeDetectionOptions,
			NULL
	);

	delete thresholdImage;
	delete bigObjectsImage;
	delete convexHullImage;
	delete preRectangleImage;

	vector<Target> *targets = new vector<Target>;
	
	int size = (int) rectangles->size();
	
	if (size == 0) {
		delete rectangles;
		return targets;		// Empty vector
	}
	
	for (int i=0; i<size; i++) {
		Target t;
		RectangleMatch r = rectangles->at(i);
		
		t.Width = r.width;
		t.Height = r.height;
		t.Rotation = r.rotation;	// Rotation from camera's horizontal axis.
		
		t.Score = r.score;
		t.TopLeft.Set(r.corner[0].x, r.corner[0].y);
		t.TopRight.Set(r.corner[1].x, r.corner[1].y);
		t.BottomRight.Set(r.corner[2].x, r.corner[2].y);
		t.BottomLeft.Set(r.corner[3].x, r.corner[3].y);
		
		t.DistanceFromCamera = CalculateDistanceBasedOnWidth(t.Width);
		targets->push_back(t);
	}
	
	delete rectangles;
	return targets;
}

AxisCamera & TargetFinderThread::GetCamera()
{
	AxisCamera &camera = AxisCamera::GetInstance(mCameraIp);
	camera.WriteResolution(AxisCamera::kResolution_320x240);
	camera.WriteCompression(20);
	camera.WriteBrightness(0);
	return camera;	
}

double TargetFinderThread::CalculateDistanceBasedOnWidth(double widthInPixels)
{
	// Based on exeriments we conducted...
	// Axis 206 Network camera
	// 640x480
	// The dial contains a focus -- the groove over the 'ar' in
	// 'Near' should be just a hair to the left of the bump.
	
	double distance = (17490 / widthInPixels) - 6.97; 
	return distance;
}
