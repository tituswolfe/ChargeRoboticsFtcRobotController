package org.firstinspires.ftc.teamcode.util;

import java.util.Map;
import java.util.TreeMap;

/**
 * {@link LinearInterpolator} Has a function to interpolate data and estimate launch angle.
 *
 * @author Jonny King
 * @author Titus
 */

public class LinearInterpolator {
    private final TreeMap<Double, Double> dataPoints;

    public LinearInterpolator(TreeMap<Double, Double> dataPoints){
        this.dataPoints = dataPoints;
    }

    /**
     * Performs linear interpolation to estimate the y-value for a given x.
     * @param x The x-value to interpolate for.
     * @return The interpolated y-value.
     */
    public double interpolate(double x){
        if (dataPoints.isEmpty()) return 0;
        if (dataPoints.containsKey(x)) return dataPoints.get(x);

        Map.Entry<Double, Double> floorEntry = dataPoints.floorEntry(x);
        Map.Entry<Double, Double> ceilingEntry = dataPoints.ceilingEntry(x);

        if (floorEntry == null) return ceilingEntry.getValue();
        if (ceilingEntry == null) return floorEntry.getValue();

        double x1 = floorEntry.getKey();
        double y1 = floorEntry.getValue();
        double x2 = ceilingEntry.getKey();
        double y2 = ceilingEntry.getValue();

        double t = (x - x1) / (x2 - x1);

        return lerp(y1, y2, t);
    }

    /**
     * Calculates the linear interpolation (lerp) between two values.
     * @param v0 The starting value (y1).
     * @param v1 The ending value (y2).
     * @param t The interpolation factor, typically between 0 and 1.
     * @return The interpolated value.
     */
    private static double lerp(double v0, double v1, double t) {
        return v0 + t * (v1 - v0);
    }
}
