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
     * <style>.eq {font-family: "Times New Roman","Symbola-mq",serif;}
     * .i {font-style: italic;}.sym {margin: 0 4px;}</style>
     * 
     * Easing function using this piecewise function of two parameterized functions<br/>
     * 
     * <div><img src="http://www.spartabots.org/uploads/2015/01/easing-function.png" style="width:375px;margin-left:50px" /></div><br/>
     * 
     * As <span class="sym i eq">a</span> increases, the slope in the middle becomes greater (steeper) and
     * the ends at <span class="eq"><span class="i">x</span><span class="sym">=</span>0</span> and
     * <span class="eq"><span class="i">x</span><span class="sym">=</span>1</span> become flatter.<br/>
     * As <span class="eq"><span class="i">a</span><span class="sym">&rarr;</span>&infin;</span>, the curve
     * tends towards a step function.
     * 
     * <br/><br/>
     * 
     * <small>From http://math.stackexchange.com/questions/121720/ease-in-out-function</small><br/><br/>
     * 
     * @param x the x value for the ease function between -1 and 1
     * @param a the a value for the ease function, must be equal to or greater than 0; 2 works well
     * @return eased x
     */
    public static double ease(double x, double a) {
    	if (x >= 0) {
    		double pXA = Math.pow(x,a);
    		return pXA/(pXA + Math.pow(1-x,a));
    	} else if (x < 0) {
    		double npXA = Math.pow(-x,a);
    		return -(npXA/(npXA + Math.pow(1+x,a)));
    	}
    	
    	return -1;
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