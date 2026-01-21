package org.firstinspires.ftc.teamcode.util.math;

import com.pedropathing.geometry.Pose;

public class MathUtil {
    public static final double TAU = Math.PI * 2; // TAU = 2𝜋

    public static double clamp(double val, double min, double max) {
        return Math.min(max, Math.max(min, val));
    }

    public static boolean isWithinRange(double val, double target, double margin) {
        return val > target - margin && val < target + margin;
    }

    public static double bearingTo(Pose initialPose, Pose targetPose) {
        Pose displacement = targetPose.minus(initialPose);
        return Math.atan2(displacement.getY(), displacement.getX());
    }
}
