#include "components.h"

BaseComponent::BaseComponent() 
{
	// Empty
}

BaseController::BaseController() 
{
	// Empty
}


/**
 * @brief Takes a number between a certain range and scales
 * it within another range.
 * 
 * @details
 * For example, if I had
 * @code
 * Tools::Coerce(3, 0, 10, 0, 100);
 * @encode,
 * 
 * The code would see that '3' is %30 percent the range
 * 0 to 10, and then return 30 percent of the range from
 * 0 to 100.
 * 
 * Examples:
 * @code
 * Tools::Coerce(3, 0, 10, 0, 100);		// 30
 * Tools::Coerce(4, 0, 10, 10, 110);	// 40
 * Tools::Coerce(4, 1, 11, 200, 400);	// 260
 * @endcode 
 * 
 * @param[in] number The number to convert
 * @param[in] rawMin The smallest number of the original range
 * @param[in] rawMax The largest number of the original range
 * @param[in] adjustedMin The smallest number of the output range
 * @param[in] adjsutedMax The largest number of the output range.
 */
double Tools::Coerce(double number, double rawMin, double rawMax, double adjustedMin, double adjustedMax)
{
	// Check inputs for validity.
	assert(rawMin <= rawMax);
	assert(adjustedMin <= adjustedMax);
	
	if (number < rawMin) {
		number = rawMin;
	} else if (number > rawMax) {
		number = rawMax;
	}
	
	double percentage = (number - rawMin) / (rawMax - rawMin);
	return percentage * (adjustedMax - adjustedMin); 
}

