package org.firstinspires.ftc.teamcode.util;

public class LogicUtil {
    public static boolean isWithinRange(double val, double target, double margin) {
        return val > target - margin && val < target + margin;
    }

    public static double clamp(double val, double min, double max) {
        return Math.min(max, Math.max(min, val));
    }


}
