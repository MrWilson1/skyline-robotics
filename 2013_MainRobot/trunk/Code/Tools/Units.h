#ifndef UNITS_H
#define UNITS_H

enum Units {
	Inches,
	Feet,
	Meters,
	Centimeters
};

float ConvertUnits(float value, Units start, Units target);


#endif
