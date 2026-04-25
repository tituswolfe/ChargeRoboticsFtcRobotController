package org.firstinspires.ftc.teamcode.util.math;

public class Conversion {
    public static double millsToSec(double mills) {
        return mills / 1000;
    }

    public static double nsToSec(long nanoseconds) {
        return nanoseconds / 1e-9;
    }
}
