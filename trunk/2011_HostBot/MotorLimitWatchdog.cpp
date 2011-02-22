

#include "MotorLimitWatchdog.h"

MotorLimitWatchdog::MotorLimitWatchdog(
		const char * watchdogName,
		SpeedController * motor,
		DigitalInput * highLimit,
		DigitalInput * lowLimit):
	Task("MotorLimitWatchDog", (FUNCPTR)MotorLimitWatchdog::TaskWrapper)
{
	m_name = watchdogName;
	m_motor = motor;
	m_highLimit = highLimit;
	m_lowLimit = lowLimit;
	
	m_statusLogger = new Notifier((TimerEventHandler)MotorLimitWatchdog::LogStatus, this);
	m_statusLogger->StartPeriodic(1.0);
	
	Task::Start((UINT32)this);
}

MotorLimitWatchdog::~MotorLimitWatchdog()
{
	if (m_statusLogger)
	{
		m_statusLogger->Stop();
		delete m_statusLogger;
	}
	
}


// static wrapper function to callback non-static member function
void
MotorLimitWatchdog::TaskWrapper(void* ThisObject)
{
	// explicit cast pointer to Class pointer
	MotorLimitWatchdog * myself = (MotorLimitWatchdog*) ThisObject;

	// call member function
	myself->Run();
}



void
MotorLimitWatchdog::LogStatus (MotorLimitWatchdog * obj)
{
	char * pStatusStr = new char[strlen(obj->m_name) + 32];
	char * pStatusText;
	
	pStatusText = strcpy (pStatusStr, obj->m_name);
	pStatusText += strlen(pStatusStr);
	strcat(pStatusText++, " ");
	
	strcpy(pStatusText, "High limit set:");
	SmartDashboard::Log((bool)(obj->m_highLimit->Get()), pStatusStr);

	strcpy(pStatusText, "Ligh limit set:");
	SmartDashboard::Log((bool)(obj->m_lowLimit->Get()), pStatusStr);

	strcpy(pStatusText, "Current motor speed:");
	SmartDashboard::Log(obj->m_motor->Get(), pStatusStr);

	delete pStatusStr;
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
