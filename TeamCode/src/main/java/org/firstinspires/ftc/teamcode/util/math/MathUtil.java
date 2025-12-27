package org.firstinspires.ftc.teamcode.util.math;

public class MathUtil {
    public static final double TAU = Math.PI * 2; // TAU = 2𝜋

    public static double clamp(double val, double min, double max) {
        return Math.min(max, Math.max(min, val));
    }

    public static boolean isWithinRange(double val, double target, double margin) {
        return val > target - margin && val < target + margin;
    }
}
