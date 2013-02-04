#ifndef DRIVE_SUBSYSTEM_H
#define DRIVE_SUBSYSTEM_H
/**
 * Implements various subsystems related to the drive.
 */

#include <string>

#include "WPILib.h"

#include "../BaseSubsystem.h"
#include "../../Misc/Tools.h"

/**
 * A base drive. All drives must be a subclass of this class.
 * 
 * Note that Holonomic and Mechanum wheel drives are currently
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
	
	virtual void Drive(float outputMagnitude, float curve) = 0;
	virtual void TankDrive(float leftValue, float rightValue) = 0;
	virtual void TankDrive(float leftValue, float rightValue, bool squaredInputs) = 0;
	virtual void ArcadeDrive(float moveValue, float rotateValue) = 0;
	virtual void ArcadeDrive(float moveValue, float rotateValue, bool squaredInputs) = 0;
	virtual void TravelDistance(float distanceInInches) = 0;
	virtual void Rotate(float degrees) = 0;
	virtual void Disable() = 0;
	virtual void Enable() = 0;
	virtual void Brake() = 0;
};

class IPidDrive {
public:
	IPidDrive();
	virtual ~IPidDrive();
	virtual void AdjustRatePid(float lp, float li, float ld, float rp, float ri, float rd) = 0;
};

/**
 * The absolute simplest drive possible. Contains
 * no PID loops or other forms of feedback systems.
 */
class SimpleDrive : public BaseDrive {
public:
	SimpleDrive(RobotDrive *robotDrive);
	~SimpleDrive();
	
	void Drive(float outputMagnitude, float curve);
	void TankDrive(float leftValue, float rightValue);
	void TankDrive(float leftValue, float rightValue, bool squaredInputs);
	void ArcadeDrive(float moveValue, float rotateValue);
	void ArcadeDrive(float moveValue, float rotateValue, bool squaredInputs);
	void TravelDistance(float distanceInInches);
	void Rotate(float degrees);
	void Disable();
	void Enable();
	void Brake();
	
private:
	RobotDrive *m_robotDrive;
};



/**
 * Returns running average of size `maxSize`
 * when calling PIDGet(). The type (rate or distance) must
 * be specified beforehand by configuring the encoder.
 * 
 * I could have inherited from Encoder, but I wanted to
 * keep it simple. The various config stuff of the 
 * Encoder are exposed through GetEncoder().
 */
class SmoothEncoder : public PIDSource {
public:
	SmoothEncoder(Encoder *encoder, unsigned int maxSize);
	virtual ~SmoothEncoder();
	
	double PIDGet();
	Encoder *GetEncoder();
	void SetMaxSize(unsigned int maxSize);
	
private:
	Encoder *m_encoder;
	std::deque<double> m_history;
	unsigned int m_maxSize;
	
};

class Tread : public PIDOutput {
public:
	enum TreadPidMode {
		kRate,
		kDistance
	};
	enum Direction {
		kForward = 1,
		kReversed = -1
	};
	Tread(SpeedController *front, SpeedController *back, Direction direction);
	virtual ~Tread();
	
	//TreadPidMode GetMode();
	//void SetMode(TreadPidMode PidMode);
	void PIDWrite(float output);

	float m_last;
private:
	SpeedController *m_front;
	SpeedController *m_back;
	Direction m_direction;
	//TreadPidMode m_currentMode;
};



class PidSimpleDrive : public BaseDrive, public IPidDrive {
public:
	PidSimpleDrive(
			SpeedController *leftFront, 
			SpeedController *leftBack, 
			SpeedController *rightFront,
			SpeedController *rightBack,
			Encoder *leftEncoder,
			Encoder *rightEncoder);
	~PidSimpleDrive();
	
	void Drive(float outputMagnitude, float curve);
	void TankDrive(float leftValue, float rightValue);
	void TankDrive(float leftValue, float rightValue, bool squaredInputs);
	void ArcadeDrive(float moveValue, float rotateValue);
	void ArcadeDrive(float moveValue, float rotateValue, bool squaredInputs);
	void TravelDistance(float distanceInInches);
	void Rotate(float degrees);
	void Disable();
	void Enable();
	void Brake();
	void AdjustRatePid(float lp, float li, float ld, float rp, float ri, float rd);
	
private:
	enum PidMode {
		Rate,
		Distance
	};
	void TryToggling(PidMode mode);
	
	SpeedController *m_leftFront; 
	SpeedController *m_leftBack;
	SpeedController *m_rightFront;
	SpeedController *m_rightBack;
	Encoder *m_leftEncoder;
	Encoder *m_rightEncoder;
	
	SmoothEncoder *m_smoothLeftEncoder;
	SmoothEncoder *m_smoothRightEncoder;
	Tread *m_leftTread;
	Tread *m_rightTread;
	
	PIDController *m_leftPidRate;
	PIDController *m_rightPidRate;
	PIDController *m_leftPidDistance;
	PIDController *m_rightPidDistance;
	
	PidMode m_currentMode;
};

#endif
