#include "Units.h"

float Convert_Units(float number, Units start, Units target)
{
	float intermediateValue = 0;
	switch (start) {
	case Inches:
		intermediateValue = number;
		break;
	case Feet:
		intermediateValue = number * 12;
		break;
	case Meters:
		intermediateValue = number * 39.3701;
		break;
	case Centimeters:
		intermediateValue = number * 0.39301;
		break;
	default:
		// *shrug*
		break;
	};
	
	float endValue = 0;
	switch (target) {
	case Inches:
		endValue = intermediateValue;
		break;
	case Feet:
		endValue = intermediateValue * 12;
		break;
	case Meters:
		endValue = intermediateValue * 0.0254;
		break;
	case Centimeters:
		endValue = intermediateValue * 2.54;
		break;
	default:
		// *shrug*
		break;
	};
	
	return endValue;
	
}
