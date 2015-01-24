package org.spartabots.frc2015.util;

public class Util {
	/**
	 * Used to prevent over sensitivity on controller axis.
	 * If n is between -0.1 and 0.1, then returns 0, else returns n.
	 * 
	 * @param n A number between -1 and 1
	 * @return 0 or n
	 */
    public static double cutoff(double n) {
        if ((-0.1 <= n) && (n <= 0.1)) {
        	return 0;
        }
        return n;
    }
    
    /**
     * Limit between lower bound and higher bound
     * 
     * @param value
     * @param lowLimit
     * @param highLimit
     * @return
     */
    public static double limit(double value, double lowerBound, double higherBound) {
        if (value < lowerBound) {
            value = lowerBound;
        }
        if (value > higherBound) {
            value = higherBound;
        }
        return value;
    }
    
    /**
     * Limit between -1 and 1
     * 
     * @param value
     * @return
     */
    public static double limit1(double value) {
        if (value < -1) {
            value = -1;
        }
        if (value > 1) {
            value = 1;
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
    
    public static boolean isWithinBounds(double value, double lowerBound, double higherBound) {
        return value <= higherBound && value > lowerBound;
    }
    
    public static boolean isWithinRange(double input, double target, double range) {
    	double offset = target - input;
        if (offset < 0.0) {
            offset = -offset;
        }
        return offset <= range;
    }
}