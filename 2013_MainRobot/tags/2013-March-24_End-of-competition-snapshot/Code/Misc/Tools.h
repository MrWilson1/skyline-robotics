#ifndef TOOLS_H
#define TOOLS_H
/**
 * \file Tools.h
 * 
 * \brief Contains utility functions. See the [Tools](/ref tools) module 
 * 
 * \defgroup tools Tools
 * 
 * \brief Contains utility functions.
 * 
 * Contains a bunch of misc functions (usually mathematical in nature) 
 * to assist with various snippits of code.
 * 
 * \see Tools.h 
 * 
 * \addtogroup tools
 * \{
 */

#include <math.h>

#include "WPILib.h"

/**
 * \brief Contains all functions within the [Tools module](/ref tools)
 */
namespace Tools {
	const double PI = 3.14159;
	/**
	 * \brief An enum for various units (for unit conversion)
	 * 
	 * \ingroup tools
	 */
	enum Units {
		kInches,
		kFeet,
		kMeters,
		kCentimeters
	};
	
	/**
	 * \brief Converts number in the START unit to the TARGET unit.
	 * 
	 * \ingroup tools
	 */
	float ConvertUnits(float value, Units start, Units target);
	
	/**
	 * \brief Makes sure that a number lies between the range 
	 * [lowLimit] and [highLimit]. 
	 * 
	 * If not, it forces it toconform to that range.
	 * 
	 * \ingroup tools
	 */
	float Limit(float value, float lowLimit, float highLimit);
	
	/**
	 * \brief Scales a value from the range [inputStart, inputEnd] to the range 
	 * [outputStart, outputEnd]
	 * 
	 * \ingroup tools
	 */
	float Scale(float value, float inputStart, float inputEnd, float outputStart, float outputEnd);
	
	/**
	 * \brief Finds the magnitude of two coordinate pairs
	 * 
	 * \ingroup tools
	 */
	float FindMagnitude(float a, float b);
	
	/**
	 * \brief Squares the number, but makes the output negative if the original was
	 * negative
	 * 
	 * \ingroup tools
	 */
	float SquareMagnitude(float value);
	
	/**
	 * \brief Returns true if the input number is within a certain range of
	 * a target number (in both the positive and negative range).
	 * 
	 * \ingroup tools
	 */
	bool IsWithinRange(float input, float target, float range);
	
	/**
	 * \brief If a number is within a certain range of zero, outputs zero. Otherwise,
	 * the original number is returned.
	 */
	double Deadband(double input, double deadband);
	
	/**
	 * \brief cmath's sin() function, but with degrees instead of radians.
	 */
	double DegSin(double angle);
	
	/**
	 * \brief cmath's cos() function, but with degrees instead of radians.
	 */
	double DegCos(double angle);
	
	/**
	 * \brief cmath's tan() function, but with degrees instead of radians.
	 */
	double DegTan(double angle);
	
	/**
	 * \brief cmath's asin() function, but with degrees instead of radians.
	 */
	double DegAsin(double value);
	
	/**
	 * \brief cmath's acos() function, but with degrees instead of radians.
	 */
	double DegAcos(double value);
	
	/**
	 * \brief cmath's atan() function, but with degrees instead of radians.
	 */
	double DegAtan(double value);
	
	/**
	 * \brief Using three sides of a triangle, calculates the angle opposite to the parameter angleSide with the law of cosines.
	 */
	double FindAngleOnTriangle(double side1, double side2, double angleSide);
	
	/**
	 * \brief Adds a robot preference with the default value, but only if it doesn't exist.
	 */
	void TrySaveString (const char *key, const char *value);
	void TrySaveInt (const char *key, int value);
	void TrySaveDouble (const char *key, double value);
	void TrySaveFloat (const char *key, float value);
	void TrySaveBoolean (const char *key, bool value);
	void TrySaveLong (const char *key, INT64 value);  

	/**
	 * \brief Checks if a value is between two given values (a higher bound and a lower bound).
	 */
	bool IsWithinBounds(double higherBound, double lowerBound, double value);
}

/**
 * \}
 */
#endif
