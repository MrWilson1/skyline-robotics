package org.spartabots.frc2015.util;

import edu.wpi.first.wpilibj.Joystick;

public class XboxController extends Joystick {

    public static class Axis {

        public static final int LeftX = 0;
        public static final int LeftY = 1;
        public static final int LeftTrigger = 2;
        public static final int RightTrigger = 3;
        public static final int RightX = 4;
        public static final int RightY = 5;
    }

    public static class Dpad {

        public static final int North = 0;
        public static final int NorthEast = 45;
        public static final int East = 90;
        public static final int SouthEast = 135;
        public static final int South = 180;
        public static final int SouthWest = 225;
        public static final int West = 270;
        public static final int NorthWest = 315;
        public static final int Center = -1;
    }

    public static class Button {

        public static final int A = 1;
        public static final int B = 2;
        public static final int X = 3;
        public static final int Y = 4;
        public static final int LeftBumper = 5;
        public static final int RightBumper = 6;
        public static final int Back = 7;
        public static final int Start = 8;
        public static final int LeftStickClick = 9;
        public static final int RightStickClick = 10;
    }

    static final int numAxisTypes = 7;
    static final int numButtonTypes = 10;

    public XboxController(int port) {
        super(port, numAxisTypes, numButtonTypes);
    }

    /**
     * Gets the value of an axis of the specified axis, ranging from -1 to 1
     * 
     * @param axisName axis
     * @return axis value
     */
    public double getAxis(int axisName) {
        return getRawAxis(axisName);
    }

    /**
     * Gets the value of the dpad
     * 
     * @param dpad dpad
     * @return dpad value
     */
    public int getDpad() {
    	return this.getPOV();
    }

    /**
     * Gets if the specified button is pressed.
     * 
     * @param buttonName button
     * @return button value
     */
    public boolean getButton(int buttonName) {
        return getRawButton(buttonName);
    }

    public double getLeftXAxis() {
        return getAxis(Axis.LeftX);
    }

    public double getLeftYAxis() {
        return getAxis(Axis.LeftY);
    }
    
    public double getLeftTriggerAxis() {
        return getAxis(Axis.LeftTrigger);
    }

    public double getRightTriggerAxis() {
        return getAxis(Axis.RightTrigger);
    }

    public double getRightXAxis() {
        return getAxis(Axis.RightX);
    }

    public double getRightYAxis() {
        return getAxis(Axis.RightY);
    }

    public boolean getAButton() {
        return getButton(Button.A);
    }

    public boolean getBButton() {
        return getButton(Button.B);
    }

    public boolean getXButton() {
        return getButton(Button.X);
    }

    public boolean getYButton() {
        return getButton(Button.Y);
    }

    public boolean getLeftBumperButton() {
        return getButton(Button.LeftBumper);
    }

    public boolean getRightBumperButton() {
        return getButton(Button.RightBumper);
    }

    public boolean getBackButton() {
        return getButton(Button.Back);
    }

    public boolean getStartButton() {
        return getButton(Button.Start);
    }

    public boolean getLeftStickClickButton() {
        return getButton(Button.LeftStickClick);
    }

    public boolean getRightStickClickButton() {
        return getButton(Button.RightStickClick);
    }

}