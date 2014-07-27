#include "WPILib.h"

class Shooter
{
protected:
	Jaguar *mTopLeftSpeedController;
	Jaguar *mTopRightSpeedController;
	Jaguar *mBottomLeftSpeedController;
	Jaguar *mBottomRightSpeedController;
	
	static const float kReductionFactor = 0.9;

public:
    Shooter(Jaguar*, Jaguar*, Jaguar*, Jaguar*);
    void SetSpeed(float);
    void SetSpeed(float, float);
};
