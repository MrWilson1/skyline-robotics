package org.spartabots.frc2015;

public class Ports {
    /**
     * Mappings for the digital sidecar (digital values)
     * 
     * Pwm: Pulse-width modulation: used primarily for motors and servos
     * 
     * Dio/Gpio: General purpose input/output: used mostly for sensors
     * 
     * Relay: Used mostly for pneumatics (Spikes, etc)
     * 
     * Analog: Used mostly for analog sensors
     * 
     * Pcm: Used mostly for pneumatics
     */
    public static class RoboRIO {
        public static final int Pwm0 = 0;
        public static final int Pwm1 = 1;
        public static final int Pwm2 = 2;
        public static final int Pwm3 = 3;
        public static final int Pwm4 = 4;
        public static final int Pwm5 = 5;
        public static final int Pwm6 = 6;
        public static final int Pwm7 = 7;
        public static final int Pwm8 = 8;
        public static final int Pwm9 = 9;

        public static final int Relay0 = 0;
        public static final int Relay1 = 1;
        public static final int Relay2 = 2;
        public static final int Relay3 = 3;

        public static final int AnalogChannel0 = 0;
        public static final int AnalogChannel1 = 1;
        public static final int AnalogChannel2 = 2;
        public static final int AnalogChannel3 = 3;

        public static final int Pcm0 = 0;
        public static final int Pcm1 = 1;
        public static final int Pcm2 = 2;
        public static final int Pcm3 = 3;
        public static final int Pcm4 = 4;
        public static final int Pcm5 = 5;
        public static final int Pcm6 = 6;
        public static final int Pcm7 = 7;
        
        public static final int Dio0 = 0;
        public static final int Dio1 = 1;
        public static final int Dio2 = 2;
        public static final int Dio3 = 3;
        public static final int Dio4 = 4;
        public static final int Dio5 = 5;
        public static final int Dio6 = 6;
        public static final int Dio7 = 7;
        public static final int Dio8 = 8;
        public static final int Dio9 = 9;
                
    }
    
    /**
     * Mappings on the controller computer
     * 
     * Usb: Used mostly for joysticks (or other forms of input)
     */
    public static class Computer {
        public static final int Usb0 = 0;
        public static final int Usb1 = 1;
        public static final int Usb2 = 2;
        public static final int Usb3 = 3;
        public static final int Usb4 = 4;
    }
}
