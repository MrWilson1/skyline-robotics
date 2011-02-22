
#ifndef MOTOR_LIMIT_WATCHDOG_H_
#define MOTOR_LIMIT_WATCHDOG_H_

#include "WPILib.h"


class MotorLimitWatchdog : public Task
{
	const char * m_name;
	SpeedController * m_motor;
	DigitalInput * m_highLimit;
	DigitalInput * m_lowLimit;
	Notifier * m_statusLogger;

	public:
		MotorLimitWatchdog(
				const char * watchdogName,
				SpeedController * motor,
				DigitalInput * highLimit,
				DigitalInput * lowLimit);

		~MotorLimitWatchdog();

	private:
		static void TaskWrapper (void *);
		static void LogStatus (MotorLimitWatchdog * watchdog);

		void Run();
};

#endif // MOTOR_LIMIT_WATCHDOG_H_
