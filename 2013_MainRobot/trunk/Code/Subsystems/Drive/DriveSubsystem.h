#ifndef DRIVE_SUBSYSTEM_H
#define DRIVE_SUBSYSTEM_H
/**
 * \file DriveSubsystem.h
 * 
 * Implements various subsystems related to the drive.
 * 
 * \addtogroup subsystems
 * \{
 * \defgroup drive_subsystems Drive
 * \{
 */

#include <string>

#include "WPILib.h"

#include "../BaseSubsystem.h"
#include "../Components/DriveComponents.h"
#include "../../Misc/Tools.h"

/**
 * \brief A base drive. All drives must be a subclass of this class.
 * 
 * \warning Note that Holonomic and Mechanum wheel drives are currently
 * not implemented by this interface. 
 * 
 * For the definitions of what these virtual methods are meant to
 * do, please see the WPILib API documentation on the 
 * RobotDrive class. The methods are identical.
 * 
 * Note that all drive systems are enabled upon instantiation.
 */
class BaseDrive : public BaseSubsystem {
public:
	BaseDrive(const char *name);
	virtual ~BaseDrive();
	
	virtual void TankDrive(float leftValue, float rightValue) = 0;
	virtual void TankDrive(float leftValue, float rightValue, bool squaredInputs) = 0;
	virtual void ResetDistanceAndRotation() = 0;
	virtual void TravelDistance(float distanceInInches) = 0;
	virtual void Rotate(float degrees) = 0;
	virtual void Disable() = 0;
	virtual void Enable() = 0;
	virtual void Brake() = 0;
};

/**
 * \brief A class which represents a complete PID controller for one tread and one encoder.
 * 
 * It provides a bunch of convenience functions used to manipulate both the PID controller, the
 * wheel, and the encoder.
 */
class IntegratedPid {
public:
	IntegratedPid(float p, float i, float d, float dpp, Encoder::PIDSourceParameter pidSource, 
				Encoder *encoder, Tread *tread);
	~IntegratedPid();
	
	void SetRaw(float speed);
	void SetSetpoint(float setpoint);
	void SetPid(float p, float i, float d);
	void SetDistancePerPulse(float dpp);
	float GetSpeed();
	void Reset();
	void Enable();
	void Disable();
	PIDController *GetController();
	float GetP();
	float GetI();
	float GetD();
	float GetDistancePerPulse();
	float GetDistance();
	
private:
	Encoder *m_encoder;
	Tread *m_tread;
	PIDController *m_pid;
	float m_p;
	float m_i;
	float m_d;
	float m_dpp;
	Encoder::PIDSourceParameter m_pidSource;
};

/**
 * \brief A class representing one complete PID controller for both threads of the robot.
 */
class DrivePid {
public:
	DrivePid(IntegratedPid leftPid, IntegratedPid rightPid);
	~DrivePid();
	
	void SetSetpoint(float left, float right);
	void Reset();
	void Enable();
	void Disable();
	
	IntegratedPid Left;
	IntegratedPid Right;
};


/**
 * \brief An interface for any PID-based drive.
 */
class IPidDrive {
public:
	IPidDrive();
	virtual ~IPidDrive();
	virtual void UpdatePidValues() = 0;
	virtual void AdjustRatePid(float lp, float li, float ld, float rp, float ri, float rd) = 0;
	virtual void AdjustDistancePid(float lp, float li, float ld, float rp, float ri, float rd) = 0;
	virtual DrivePid GetRatePid() = 0;
	virtual DrivePid GetDistancePid() = 0;
	virtual void SetRatePid(DrivePid ratePid) = 0;
	virtual void SetDistancePid(DrivePid distancePid) = 0;
};

/**
 * \brief The absolute simplest drive possible. Contains
 * no PID loops or other forms of feedback systems.
 */
class SimpleDrive : public BaseDrive {
public:
	SimpleDrive(Tread *left, Tread *right);
	~SimpleDrive();
	
	void TankDrive(float leftValue, float rightValue);
	void TankDrive(float leftValue, float rightValue, bool squaredInputs);
	void ResetDistanceAndRotation();
	void TravelDistance(float distanceInInches);
	void Rotate(float degrees);
	void Disable();
	void Enable();
	void Brake();
	
private:
	Tread *m_leftTread;
	Tread *m_rightTread;
};



/**
 * \brief A varient of an encoder which returns a running average 
 * of the past five values from PIDGet.
 * 
 * Specifically, it returns running average of size `maxSize`
 * when calling PIDGet(). The type (rate or distance) must
 * be specified beforehand by configuring the encoder.
 * 
 * I could have inherited from Encoder, but I wanted to
 * keep it simple. The various config stuff of the 
 * Encoder are exposed through GetEncoder().
 */
class SmoothEncoder : public PIDSource {
public:
	/**
	 * \param[in] encoder A pointer to the encoder
	 * \param[in] maxSize The length of the running average
	 */
	SmoothEncoder(
			Encoder *encoder, 
			unsigned int maxSize,
			Encoder::PIDSourceParameter pidSourceParameter,
			float distancePerPulse);
	virtual ~SmoothEncoder();
	
	/**
	 * \brief Completes the PIDSource interface.
	 * 
	 * Returns the rate or the distance, depending on what
	 * the encoder is set to.
	 */
	double PIDGet();
	/**
	 * \brief Returns a pointer to the internal encoder
	 */
	Encoder *GetEncoder();
	/**
	 * \brief Sets the size of the running average.
	 */
	void SetMaxSize(unsigned int maxSize);
	unsigned int GetMaxSize();
	
	void SetPIDSourceParameter(Encoder::PIDSourceParameter pidSourceParameter);
	Encoder::PIDSourceParameter GetPIDSourceParameter();
	
	void SetDistancePerPulse(float distancePerPulse);
	float GetDistancePerPulse();
	
	void Reset();
	
private:
	Encoder *m_encoder;
	std::deque<double> m_history;
	unsigned int m_maxSize;
	Encoder::PIDSourceParameter m_pidSourceParameter;
	float m_distancePerPulse;
};



/**
 * \brief A drive system using PID controllers.
 * 
 * Contains two sets of PID controllers -- one for rate, and another for distance.
 * 
 * \warning The PID loops are not yet well-calibrated, and are buggy.
 * 
 * \warning Not all methods have been implemented
 * 
 * \warning Only the rate PID loops have been calibrated
 */
class PidSimpleDrive : public BaseDrive, public IPidDrive {
public:
	PidSimpleDrive(
			DrivePid ratePid,
			DrivePid distancePid,
			double robotDiagonalLength);
	~PidSimpleDrive();
	
	void TankDrive(float leftValue, float rightValue);
	void TankDrive(float leftValue, float rightValue, bool squaredInputs);
	void ResetDistanceAndRotation();
	void TravelDistance(float distanceInInches);
	void Rotate(float degrees);
	void Disable();
	void Enable();
	void Brake();
	void UpdatePidValues();
	void AdjustRatePid(float lp, float li, float ld, float rp, float ri, float rd);
	void AdjustDistancePid(float lp, float li, float ld, float rp, float ri, float rd);
	DrivePid GetRatePid();
	DrivePid GetDistancePid();
	void SetRatePid(DrivePid ratePid); // A hack, for the transmissions.
	void SetDistancePid(DrivePid distancePid);
	
private:
	enum PidMode {
		Rate,
		Distance
	};
	void TryToggling(PidMode mode);
	PidMode m_currentMode;
	DrivePid m_ratePid;
	DrivePid m_distancePid;
	double m_robotDiagonalLength;
	
	
};

/**
 * \}
 * \}
 */
#endif
