#include "Tools.h"

float Tools::ConvertUnits(float number, Units start, Units target) {
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
		intermediateValue = number * 0.393701;
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
		endValue = intermediateValue / 12;
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

float Tools::Limit(float value, float lowLimit, float highLimit) {
	if (value < lowLimit) {
		value = lowLimit;
	}
	if (value > highLimit) {
		value = highLimit;
	}
	return value;
}

float Tools::Scale(float value, float inputStart, float inputEnd, float outputStart, float outputEnd) {
	value = Limit(value, inputStart, inputEnd);
	float percentage = (value - inputStart) / (inputEnd - inputStart);
	float scaledValue = (outputEnd - outputStart) * percentage;
	return scaledValue + outputStart;
}
