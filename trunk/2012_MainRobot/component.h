/**
 * component.h
 * 
 * Contains the basic essentials necessary to initialize
 * any subcomponent of the robot.  In specific, it provides
 * a single abstract base class ('BaseComponent') to simplify
 * functionality, and a series of typedefs corresponding to
 * every motor port, digital IO, and USB ports.
 */

#ifndef COMPONENT_H_
#define COMPONENT_H_

/**
 * BaseController
 * 
 * This class is meant to be overwritten.  It provides a 
 * convenient base class so you could easily iterate through
 * an array of components and call common methods to them.
 * 
 * In every case, the 'Run' function is meant to be called
 * in a loop.
 */
class BaseController
{
public:
	BaseController();
	virtual void Run() = 0;
};


// Namespaces:
// To use a port, use the double-colon syntax:
//     Ports::Pwm1
//     Ports::Pwm2
// For example, to declare a jaguar, you might do...
//     Jaguar *jaguar = new Jaguar(Ports::Pwm2);
// To declare a joystick, you might do...
//     Joystick *joystick = new Joystick(Ports::Usb1);
namespace Ports
{
	/**
	 * PWM Ports
	 * 
	 * Used frequently for speed-controllers (including Jaguars).
	 * Known uses:
	 *   - Jaguars for the robot drive
	 *   - Other speed controllers for motors
	 *   
	 * Found on the digital sidecar.
	 */
	enum Pwm
	{
		Pwm1 = 1,
		Pwm2 = 2,
		Pwm3 = 3,
		Pwm4 = 4,
		Pwm5 = 5,
		Pwm6 = 6,
		Pwm7 = 7,
		Pwm8 = 8,
		Pwm9 = 9,
		Pwm10 = 10
	};
	
	/**
	 * DigitalIo Ports
	 * 
	 * Used frequently for sensors.  When used with a jumper, can also
	 * be used to power servos.
	 * Known uses:
	 *   - Limit switches
	 *   - Servos
	 *   - Accelerometer/Gyro (?)
	 * 
	 * Found on the digital sidecar.
	 */
	enum DigitalIo
	{
		DigitalIo1 = 1,
		DigitalIo2 = 2,
		DigitalIo3 = 3,
		DigitalIo4 = 4,
		DigitalIo5 = 5,
		DigitalIo6 = 6,
		DigitalIo7 = 7,
		DigitalIo8 = 8,
		DigitalIo9 = 9,
		DigitalIo10 = 10,
		DigitalIo11 = 11,
		DigitalIo12 = 12,
		DigitalIo13 = 13,
		DigitalIo14 = 14
	};	
	
	/**
	 * USB Ports
	 * 
	 * Used mostly for joysticks (or other forms of input)
	 * Known uses:
	 *   - Joysticks
	 * 
	 * Found on the USB dongle that attaches to the laptop
	 */
	enum Usb
	{
		Usb1 = 1,
		Usb2 = 2,
		Usb3 = 3,
		Usb4 = 4	// Not sure if we actually have a 4th one
	};
	
	/**
	 * AnalogBreakout
	 * 
	 * Used mostly for sensors
	 * Known uses:
	 *   - Ultrasound sensor
	 * 
	 * Found on top of the first module in the cRIO.
	 */
	enum AnalogBreakout
	{
		AnalogChannel1 = 1,
		AnalogChannel2 = 2,
		AnalogChannel3 = 3,
		AnalogChannel4 = 4,
		AnalogChannel5 = 5,
		AnalogChannel6 = 6,
		AnalogChannel7 = 7,
		AnalogChannel8 = 8
	};
	
	/**
	 * Module
	 * 
	 * This is a relatively new concept, which is meant
	 * to deal with how some FRC teams have 4-slot cRIOS
	 * while others have 8-slot cRIOS.
	 * 
	 * Slot -- the physical space you can plug a module
	 * in.  Module number -- something FIRST invented.
	 * 
	 * For a 8-slot cRIO:
	 *   - Slot 1: Module 1 {Analog Module 9201)
	 *   - Slot 2: Module 1 {Digital Module 9403)
	 *   - Slot 3: Module 1 {Solenoid Module 9472)
	 *   - Slot 4: Module 2 {empty)
	 *   - Slot 5: Module 2 {Analog Module 9201)
	 *   - Slot 6: Module 2 {Digital Module 9403)
	 *   - Slot 7: Module 2 {Solenoid Module 9472)
	 *   - Slot 8: Module   {empty)
	 */
	enum Module
	{
		Module1 = 1,
		Module2 = 2
	};
}

#endif
