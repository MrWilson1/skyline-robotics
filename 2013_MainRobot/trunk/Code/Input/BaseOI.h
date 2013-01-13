#ifndef BASE_OI_H
#define BASE_OI_H

#include "WPILib.h"
#include <string>

struct TankValues
{
	float Left;
	float Right;
};


struct ArcadeValues
{
	float Move;
	float Rotate;
};


class BaseOI //: public NamedSendable
{
public:
	BaseOI(const char *name);
	virtual ~BaseOI();
	virtual std::string GetName();
	
	virtual void SetupTeleop() = 0;
	virtual TankValues GetTankValues() = 0;
	virtual ArcadeValues GetArcadeValues() = 0;
	
private:
	const std::string m_name;	
};


#endif
