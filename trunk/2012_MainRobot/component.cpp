/**
 * component.h
 * 
 * Contains the basic essentials necessary to initialize
 * any subcomponent of the robot.  In specific, it provides
 * a single abstract base class ('BaseComponent') to simplify
 * functionality, and a series of typedefs corresponding to
 * every motor port, digital IO, and USB ports.
 */

#include "component.h"

BaseController::BaseController() {
	// Does nothing
	return;
}


