package org.spartabots.frc2015;

public class Util {
    public static double cutoff(double num) {
        if ((-0.1 <= num) && (num <= 0.1)) {
                num = 0;
        }
        return num;
    }
    
    public static double limit(double value, double lowLimit, double highLimit) {
        if (value < lowLimit) {
            value = lowLimit;
        }
        if (value > highLimit) {
            value = highLimit;
        }
        return value;
    }
    
    public static double degSin(double angle) {
        return Math.sin(angle * Math.PI / 180) * 180 / Math.PI;
    }
    
    public static double degCos(double angle) {
        return Math.cos(angle * Math.PI / 180) * 180 / Math.PI;
    }
    
    public static double degTan(double angle) {
        return Math.tan(angle * Math.PI / 180) * 180 / Math.PI;
    }
    
    public static boolean isWithinBounds(double higherBound, double lowerBound, double value) {
        return value <= higherBound && value > lowerBound;
    }
    
    public static boolean isWithinRange(float input, float target, float range) {
        float offset = target - input;
        if (offset < 0.0) {
            offset = -offset;
        }
        return offset <= range;
    }
}