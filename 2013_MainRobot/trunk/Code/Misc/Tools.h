#ifndef TOOLS_H
#define TOOLS_H

#include <math.h>

namespace Tools {
	enum Units {
		Inches,
		Feet,
		Meters,
		Centimeters
	};
	
	/**
	 * Converts number in the START unit to the TARGET unit.
	 */
	float ConvertUnits(float value, Units start, Units target);
	
	/**
	 * Makes sure that a number lies between the range 
	 * [lowLimit] and [highLimit]. If not, it forces it to 
	 * conform to that range.
	 */
	float Limit(float value, float lowLimit, float highLimit);
	
	/**
	 * Scales a value from the range [inputStart, inputEnd] to the range 
	 * [outputStart, outputEnd]
	 */
	float Scale(float value, float inputStart, float inputEnd, float outputStart, float outputEnd);
	
	/**
	 * Finds the magnitude of two coordinate pairs
	 */
	float FindMagnitude(float a, float b);
	
	/**
	 * Squares the number, but makes the output negative if the original was
	 * negative
	 */
	float SquareMagnitude(float value);
	
	/**
	 * Returns true if the input number is within a certain range of
	 * a target number (in both the positive and negative range).
	 */
	bool IsWithinRange(float input, float target, float range);
}

#endif
