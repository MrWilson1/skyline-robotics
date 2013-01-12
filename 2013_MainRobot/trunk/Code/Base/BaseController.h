#ifndef BASE_CONTROLLER_H
#define BASE_CONTROLLER_H

#include "WPILib.h"
#include <string>

class BaseController : public NamedSendable
{
public:
	BaseController(const char *name);
	BaseController(const std::string *name);
	virtual ~BaseController();
	
	virtual std::string GetName();
	
private:
	const std::string *m_name;
	
	
};

#endif
