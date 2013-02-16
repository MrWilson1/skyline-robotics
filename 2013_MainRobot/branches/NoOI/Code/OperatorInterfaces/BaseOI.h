#ifndef BASE_OI_H
#define BASE_OI_H

#include "WPILib.h"
#include <string>

/**
 * Represents the speeds of the two sides of the 
 * robot in tank drive.
 */
struct TankValues {
	float Left;
	float Right;
};

/**
 * Represents the magnitude and rotational factor of the
 * robot in arcade drive. 
 */
struct ArcadeValues {
	float Move;
	float Rotate;
};

/**
 * A class representing a base Operator Interface. Initializes 
 * Commands and binds them to buttons.
 */
class BaseOI {
public:
	BaseOI(const char *name);
	virtual ~BaseOI();
	virtual std::string GetName();
	
	// Setup for autonomous should be independent of the OI.
	/*virtual void SetupTeleop() = 0;
	virtual TankValues GetTankValues() = 0;
	virtual ArcadeValues GetArcadeValues() = 0;
	virtual float GetAxis(int axis) = 0;
	virtual float GetAxis(int axis, int joystickNumber) = 0;
	*/
protected:
	const std::string m_name;	
};


#endif
