#include "WPILib.h"
#include "Ports.h"
#include "SmartDashboard/SmartDashboard.h"
#include "Shooter.h"

#include "Controllers/XboxController.h"

class MainRobot : public SimpleRobot
{
private:
	RobotDrive *m_drive;
	XboxController *driveController;
	//XboxController *shootController;
	
	Jaguar* topLeftShooter;
	Jaguar* topRightShooter;
	Jaguar* bottomLeftShooter;
	Jaguar* bottomRightShooter;
	Jaguar* elevator;
	//Jaguar* arm;
	
	Shooter* shooter;
	
public:
	MainRobot();
	void RobotInit();
	void InitializeHardware();
	void InitializeSoftware();
	void Autonomous();
	void OperatorControl();
	void Test();
	float Cutoff(float num);
};
