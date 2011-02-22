
#ifndef MOTOR_LIMIT_WATCHDOG_H_
#define MOTOR_LIMIT_WATCHDOG_H_

#include "WPILib.h"


class MotorLimitWatchdog : public Task
{
	SpeedController * m_motor;
	DigitalInput * m_highLimit;
	DigitalInput * m_lowLimit;

	public:
		MotorLimitWatchdog(
				SpeedController * motor,
				DigitalInput * highLimit,
				DigitalInput * lowLimit);

		~MotorLimitWatchdog();
		
		bool Start ();
		void Run();

};



#endif // MOTOR_LIMIT_WATCHDOG_H_
