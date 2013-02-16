#ifndef BASE_OI_H
#define BASE_OI_H

#include "WPILib.h"
#include <string>


/**
 * A class representing a base Operator Interface. Initializes 
 * Commands and binds them to buttons.
 */
class BaseOI {
public:
	BaseOI(const char *name);
	virtual ~BaseOI();
	virtual std::string GetName();
	
protected:
	const std::string m_name;	
};


#endif
