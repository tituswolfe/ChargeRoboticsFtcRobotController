package org.firstinspires.ftc.teamcode.util;

public class LogicUtil {
    public static boolean isWithinRange(double val, double target, double margin) {
        return val > target - margin && val < target + margin;
    }
}
