#ifndef MINIBOT_DEPLOYMENT_H_
#define MINIBOT_DEPLOYMENT_H_

#include "WPILib.h"

class MinibotDeployment
{
	// Deployment constants
	static const float MINIBOT_DEPLOY_SPEED = 0.3;

	private:
		SpeedController * m_deployMotor;	// The motor that deploys the minibot
		DigitalInput * m_deployFarLimit;	// The outer limit for minibot deployment
		DigitalInput * m_deployNearLimit;	// The closer limit for minibot.

	public:


	public:
		MinibotDeployment (
				UINT32 motorPort,
				UINT32 deployedSwitchPort,
				UINT32 retractedSwitchPort);
		~MinibotDeployment ();
		
		bool deploy();
		bool retract();

};


#endif	// MINIBOT_DEPLOYMENT_H_
