/**
 * \file Experimental.h
 * 
 * \brief A file containing short experimental subsystems or subsystems
 * used for testing. See the [Experimental](\ref experimental_subsystems) module.
 *
 * \defgroup experimental_subsystems Experimental
 * 
 * \brief A file containing short experimental subsystems or subsystems
 * used for testing. See the [Experimental](\ref experimental_subsystems) module.
 *
 * This module is essentially a dump bucket for any misc subsystems or 
 * anything which really doesn't have a home. For now, it also contains
 * any subsystems which help diagnose and analyze the output of 
 * various sensors.
 * 
 * \see Experimental.h
 * 
 * 
 * \addtogroup subsystems
 * \{
 * \addtogroup experimental_subsystems
 * \{
 */

#ifndef EXPERIMENTAL_H
#define EXPERIMENTAL_H

#include "WPILib.h"
#include "NetworkCommunication/UsageReporting.h"
#include "BaseSubsystem.h"

/**
 * \brief Controls a single motor.
 */
class TestMotor : public BaseSubsystem {
public:
	/**
	 * This requires an arbitrary name, so multiple instances
	 * of this subsystem can safely coexist in the SmartDashboard.
	 */
	TestMotor(SpeedController *motor, const char *name);
	~TestMotor();
	
	/**
	 * \brief Sets the speed of the motor to the given speed.
	 * 
	 * \param[in] speed The speed from -1.0 (reverse) to 1.0 (forward)
	 */
	void SetSpeed(float speed);
	/**
	 * \brief Sets the speed of the motor to 1.0 
	 */
	void MaxSpeed();
	
public:
	SpeedController *m_motor;
};

/**
 * \brief Reports all telemetry from an encoder, and provides hooks to 
 * configure it during runtime.
 * 
 * See the WPILib encoder class for information on what the 'Set' methods
 * do.
 */
class TestEncoder : public BaseSubsystem {
public:
	/**
	 * \param[in] encoder A pointer to the encoder to measure
	 * \param[in] name An arbitrary name, so multiple instances
	 * of this subsystem can safely coexist in the SmartDashboard.
	 * 
	 * The distance per pulse is set to a default 1.0f/4000.0f
	 */
	TestEncoder(Encoder *encoder, const char *name);
	TestEncoder(Encoder *encoder, float distancePerPulse, const char *name);
	~TestEncoder();
	
	/**
	 * \brief Writes all encoder values to the SmartDashboard
	 */
	void ReportAll();
	void Start();
	void Stop();
	void Reset();
	void SetMaxPeriod(double maxPeriod);
	void SetMinRate(double minRate);
	void SetDistancePerPulse(double input);
	void SetReverseDirection(bool reverseDirection);
	void SetPIDSourceParameter(Encoder::PIDSourceParameter pidSource);
	
	double GetDistancePerPulse();
	
private:
	/**
	 * \brief Logs a given value to the SmartDashboard
	 */
	void Log(const char *key, int value);
	/**
	 * \brief Logs a given value to the SmartDashboard
	 */
	void Log(const char *key, double value);
	/**
	 * \brief Logs a bool to the SmartDashboard
	 * 
	 * I wanted to name this 'Log' also, and take advantage of 
	 * function overloading, but for some reason, doing so caused
	 * the robot to crash.
	 */
	void LogBool(const char *key, bool value);
	Encoder *m_encoder;
	// Manually keeping track because there's no way to get it directly
	// from the Encoder.
	double m_distancePerPulse;
};


/**
 * IFI Victor Speed Controller (reversed)
 */
class ReversedVictor : public SafePWM, public SpeedController
{
public:
	explicit ReversedVictor(UINT32 channel);
	ReversedVictor(UINT8 moduleNumber, UINT32 channel);
	virtual ~ReversedVictor();
	virtual void Set(float value, UINT8 syncGroup=0);
	virtual float Get();
	virtual void Disable();

	virtual void PIDWrite(float output);

private:
	void InitVictor();
};


/** 
 * \} 
 * \} 
 */
#endif
