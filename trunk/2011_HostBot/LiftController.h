#ifndef LIFT_CONTROLLER_H
#define LIFT_CONTROLLER_H

#include "WPILib.h"
#include "TypeDefs.h"

int GetSign(float numberInput);

class LiftController
{
	public:
		typedef enum
		{
			PRESET_BOTTOM,
			PRESET_PEG1,
			PRESET_PEG2,
			PRESET_PEG3
		} PRESETS;

	private:
		SpeedController * m_liftMotor;	// Controls the lift
		DigitalInput * m_highLimit;		// The high limit for the lift
		DigitalInput * m_lowLimit;		// The lower limit for the lift
		PRESETS	m_currentPreset;		// The currently selected preset
		float	m_currentHeight;		// The current height of the lift in inches
		float   m_arrayOfHeights[5];	// The various heights for the lift.
		
	public:
		// Button assignments (Scissor-lift)
		static const UINT32 PRESET_BOTTOM_BUTTON = kJSButton_2;	// Botton top button
		static const UINT32 PRESET_PEG_1_BUTTON = kJSButton_4;		// Left top button
		static const UINT32 PRESET_PEG_2_BUTTON = kJSButton_3; 	// Center top button
		static const UINT32 PRESET_PEG_3_BUTTON = kJSButton_5; 	// Right top button
		
		static const float TURN_TRANSFORM = 0.5;		// Debug value
		// TURN_TRANSFORM:	Transforms the wanted distance to the correct amount
		// 					of motor rotations.
		// 					To use:
		// 					Rotation = Distance / TURN_TRANSFORM;
		// 					Distance = Rotation * TURN_TRANSFORM;
		// 					Partially verified value.
		
		static const float ROBOT_HEIGHT 	= 36.5;		// More accurate.
		// ROBOT_HEIGHT:	Measures from the floor to the height of the scissors
		// 					when fully compressed, in inches.  


	public:
		LiftController (
				UINT32 motorPort,
				UINT32 highLimitPort,
				UINT32 lowLimitPort);
		~LiftController ();

		bool isAtTop();
		bool isAtBottom();

		bool stop();
		bool extend(float speed);
		bool retract(float speed);
		
		bool isPresetSelected(GenericHID * inputDevice);
		TriState moveToPreset();
		TriState moveToPeg(PRESETS preset);
		
		float getCurrentHeight();
		

};


#endif	// LIFT_CONTROLLER_H
