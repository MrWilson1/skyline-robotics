

#include "MotorLimitWatchdog.h"

LOCAL void
MotorLimitWatchdogFunc(MotorLimitWatchdog * pWatchdog)
{
	pWatchdog->Run();
}


MotorLimitWatchdog::MotorLimitWatchdog(
		SpeedController * motor,
		DigitalInput * highLimit,
		DigitalInput * lowLimit):
	Task("MotorLimitWatchDog", (FUNCPTR)MotorLimitWatchdogFunc)
{
	m_motor = motor;
	m_highLimit = highLimit;
	m_lowLimit = lowLimit;
}

MotorLimitWatchdog::~MotorLimitWatchdog()
{
	
}


bool
MotorLimitWatchdog::Start()
{
	return Task::Start((UINT32)this);
}

void
MotorLimitWatchdog::Run()
{
	while(this->Verify())
	{
		if (m_highLimit->Get() || m_lowLimit->Get())
			m_motor->Set(0);
		Wait(0.5);
	}
}
