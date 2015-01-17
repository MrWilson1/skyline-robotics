package org.spartabots.frc2015.controller;

import edu.wpi.first.wpilibj.Joystick;

public class XboxController extends Joystick {

    public static class Axis {

        public static final int LeftX = 1;
        public static final int LeftY = 2;
        public static final int Trigger = 3;
        public static final int RightX = 4;
        public static final int RightY = 5;
    }

    public static class Dpad {

        public static final int XDir = 6;
        public static final int YDir = 7;
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
     * Gets the value of the dpad direction. Y-dir: - Top = 1 - Bottom = -1 X-dir: - Top = 1 - Bottom = -1
     * 
     * @param dpad dpad
     * @return dpad value
     */
    public int getDpad(int dpad) {
        return (int) getRawAxis(dpad);
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

    /**
     * Returns the trigger value. The triggers are the two trigger-like things.
     * If the left one is fully pressed, this returns 1. If the right one is fully pressed,
     * this returns -1. If both are pressed, the sum of both is the outputted value.
     *
     * @return trigger value
     */
    public double getTriggerAxis() {
        return getAxis(Axis.Trigger);
    }

    public double getRightXAxis() {
        return getAxis(Axis.RightX);
    }

    public double getRightYAxis() {
        return getAxis(Axis.RightY);
    }

    public int getXDirDpad() {
        return getDpad(Dpad.XDir);
    }

    public int getYDirDpad() {
        return getDpad(Dpad.YDir);
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