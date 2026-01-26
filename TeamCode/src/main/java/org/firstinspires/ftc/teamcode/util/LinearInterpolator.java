package org.firstinspires.ftc.teamcode.util;

import org.firstinspires.ftc.teamcode.util.math.MathUtil;

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

        return MathUtil.lerp(y1, y2, t);
    }
}
