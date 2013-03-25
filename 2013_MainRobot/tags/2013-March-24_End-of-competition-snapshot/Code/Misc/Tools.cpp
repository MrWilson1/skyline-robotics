#include "Tools.h"

float Tools::ConvertUnits(float number, Units start, Units target) {
	float intermediateValue = 0;
	switch (start) {
	case kInches:
		intermediateValue = number;
		break;
	case kFeet:
		intermediateValue = number * 12;
		break;
	case kMeters:
		intermediateValue = number * 39.3701;
		break;
	case kCentimeters:
		intermediateValue = number * 0.393701;
		break;
	default:
		// *shrug*
		break;
	};
	
	float endValue = 0;
	switch (target) {
	case kInches:
		endValue = intermediateValue;
		break;
	case kFeet:
		endValue = intermediateValue / 12;
		break;
	case kMeters:
		endValue = intermediateValue * 0.0254;
		break;
	case kCentimeters:
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

float Tools::FindMagnitude(float a, float b) {
	return sqrt(a * a + b * b);
}

float Tools::SquareMagnitude(float value) {
	float squared = value * value;
	if (value >= 0.0) {
		return squared;
	} else {
		return -squared;
	}
}

bool Tools::IsWithinRange(float input, float target, float range) {
	float offset = target - input;
	if (offset < 0.0) {
		offset = -offset;
	}
	return offset <= range;
}

double Tools::Deadband(double input, double deadband) {
	if (Tools::IsWithinRange(input, 0, deadband)) {
		return 0;
	} else {
		return input;
	}
}

double Tools::DegSin(double angle) {
	return sin(angle * Tools::PI / 180) * 180 / Tools::PI;
}
	
double Tools::DegCos(double angle) {
	return cos(angle * Tools::PI / 180) * 180 / Tools::PI;
}

double Tools::DegTan(double angle) {
	return tan(angle * Tools::PI / 180) * 180 / Tools::PI;
}

double Tools::DegAsin(double value) {
	return asin(value) * 180 / Tools::PI;
}

double Tools::DegAcos(double value) {
	return acos(value) * 180 / Tools::PI;
}

double Tools::DegAtan(double value) {
	return atan(value) * 180 / Tools::PI;
}

double Tools::FindAngleOnTriangle(double side1, double side2, double angleSide) {
	return Tools::DegAcos((pow(angleSide, 2) - pow(side1, 2) - pow(side2, 2))/(-2 * side1 * side2));
}

void Tools::TrySaveString (const char *key, const char *value) {
	Preferences *p = Preferences::GetInstance();
	if (!p->ContainsKey(key)) {
		p->PutString(key, value);
	}
}

void Tools::TrySaveInt (const char *key, int value) {
	Preferences *p = Preferences::GetInstance();
	if (!p->ContainsKey(key)) {
		p->PutInt(key, value);
	}
}

void Tools::TrySaveDouble (const char *key, double value) {
	Preferences *p = Preferences::GetInstance();
	if (!p->ContainsKey(key)) {
		p->PutDouble(key, value);
	}
}

void Tools::TrySaveFloat (const char *key, float value) {
	Preferences *p = Preferences::GetInstance();
	if (!p->ContainsKey(key)) {
		p->PutFloat(key, value);
	}
}

void Tools::TrySaveBoolean (const char *key, bool value) {
	Preferences *p = Preferences::GetInstance();
	if (!p->ContainsKey(key)) {
		p->PutBoolean(key, value);
	}
}

void Tools::TrySaveLong (const char *key, INT64 value) {
	Preferences *p = Preferences::GetInstance();
	if (!p->ContainsKey(key)) {
		p->PutLong(key, value);
	}
}

bool Tools::IsWithinBounds(double higherBound, double lowerBound, double value) {
	return value <= higherBound && value > lowerBound;
}
