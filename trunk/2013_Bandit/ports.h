#ifndef PORTS_H_
#define PORTS_H_

namespace Ports {
	namespace DigitalSidecar {
		enum Pwm {
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
		
		enum Gpio {
			Gpio1 = 1,
			Gpio2 = 2,
			Gpio3 = 3,
			Gpio4 = 4,
			Gpio5 = 5,
			Gpio6 = 6,
			Gpio7 = 7,
			Gpio8 = 8,
			Gpio9 = 9,
			Gpio10 = 10,
			Gpio11 = 11,
			Gpio12 = 12,
			Gpio13 = 13,
			Gpio14 = 14
		};
		
		enum Relay {
			Relay1 = 1,
			Relay2 = 2,
			Relay3 = 3,
			Relay4 = 4,
			Relay5 = 5,
			Relay6 = 6,
			Relay7 = 7,
			Relay8 = 8
		};
	};
	
	enum Usb {
		Usb1 = 1,
		Usb2 = 2,
		Usb3 = 3,
		Usb4 = 4
	};
	
	namespace Crio {
		enum AnalogBreakout {
			AnalogBreakout1 = 1,
			AnalogBreakout2 = 2,
			AnalogBreakout3 = 3,
			AnalogBreakout4 = 4,
			AnalogBreakout5 = 5,
			AnalogBreakout6 = 6,
			AnalogBreakout7 = 7,
			AnalogBreakout8 = 8
		};
		
		enum DigitalBreakout {
			DigitalBreakout1 = 1,
			DigitalBreakout2 = 2,
			DigitalBreakout3 = 3,
			DigitalBreakout4 = 4,
			DigitalBreakout5 = 5,
			DigitalBreakout6 = 6,
			DigitalBreakout7 = 7,
			DigitalBreakout8 = 8
		};
		
		enum Module {
			Module1 = 1,
			Module2 = 2
		};
	}
}

#endif
