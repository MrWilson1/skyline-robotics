package org.spartabots.robot2012;

public class Ports {
    /**
     * Mappings for the digital sidecar (digital values)
     * 
     * Pwm: Pulse-width modulation: used primarily for motors and servos
     * 
     * Gpio: General purpose input/output: used mostly for sensors
     * 
     * Relay: Used mostly for pneumatics (Spikes, etc)
     */
    public static class DigitalSidecar {
        public static final int Pwm1 = 1;
        public static final int Pwm2 = 2;
        public static final int Pwm3 = 3;
        public static final int Pwm4 = 4;
        public static final int Pwm5 = 5;
        public static final int Pwm6 = 6;
        public static final int Pwm7 = 7;
        public static final int Pwm8 = 8;
        public static final int Pwm9 = 9;
        public static final int Pwm10 = 10;
        
        public static final int Gpio1 = 1;
        public static final int Gpio2 = 2;
        public static final int Gpio3 = 3;
        public static final int Gpio4 = 4;
        public static final int Gpio5 = 5;
        public static final int Gpio6 = 6;
        public static final int Gpio7 = 7;
        public static final int Gpio8 = 8;
        public static final int Gpio9 = 9;
        public static final int Gpio10 = 10;
        public static final int Gpio11 = 11;
        public static final int Gpio12 = 12;
        public static final int Gpio13 = 13;
        public static final int Gpio14 = 14;
        
        public static final int Relay1 = 1;
        public static final int Relay2 = 2;
        public static final int Relay3 = 3;
        public static final int Relay4 = 4;
        public static final int Relay5 = 5;
        public static final int Relay6 = 6;
        public static final int Relay7 = 7;
        public static final int Relay8 = 8;
                
    }
    
    /**
     * Mappings on the controller computer
     * 
     * Usb: Used mostly for joysticks (or other forms of input)
     */
    public static class Computer {
        public static final int Usb1 = 1;
        public static final int Usb2 = 2;
        public static final int Usb3 = 3;
        public static final int Usb4 = 4;
    }
    
    /**
     * Additional mappings for the breakouts on the cRIO
     * 
     * Analog: Used mostly for analog sensors
     * 
     * Soleniod Breakout: Used mostly for pneumatics
     * 
     * Module: Sections of the cRIO
     * 
     * This is a relatively new concept, which is meant
     * to deal with how some FRC teams have 4-slot cRIOS
     * while others have 8-slot cRIOS
     * 
     * Slot: the physical space you can plug a module
     * in.  Module number -- something FIRST invented.
     * 
     * For a 8-slot cRIO:
     * -   Slot 1: Module 1 {Analog Module 9201)
     * -   Slot 2: Module 1 {Digital Module 9403)
     * -   Slot 3: Module 1 {Solenoid Module 9472)
     * -   Slot 4: Module 2 {empty)
     * -   Slot 5: Module 2 {Analog Module 9201)
     * -   Slot 6: Module 2 {Digital Module 9403)
     * -   Slot 7: Module 2 {Solenoid Module 9472)
     * -   Slot 8: Module   {empty)
     */
    public static class Crio {
        public static final int AnalogChannel11 = 8;
        public static final int AnalogChannel12 = 8;
        public static final int AnalogChannel13 = 8;
        public static final int AnalogChannel14 = 8;
        public static final int AnalogChannel15 = 8;
        public static final int AnalogChannel16 = 8;
        public static final int AnalogChannel17 = 8;
        public static final int AnalogChannel18 = 8;
        
        public static final int SoleniodBreakout1 = 1;
        public static final int SoleniodBreakout2 = 2;
        public static final int SoleniodBreakout3 = 3;
        public static final int SoleniodBreakout4 = 4;
        public static final int SoleniodBreakout5 = 5;
        public static final int SoleniodBreakout6 = 6;
        public static final int SoleniodBreakout7 = 7;
        public static final int SoleniodBreakout8 = 8;
        
        public static final int Module1 = 1;
        public static final int Module2 = 2;
    }
}