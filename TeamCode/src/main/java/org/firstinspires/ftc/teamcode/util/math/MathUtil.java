package org.firstinspires.ftc.teamcode.util.math;

import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;

import java.util.concurrent.TimeUnit;

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

    /**
     * Calculates the linear interpolation (lerp) between two values.
     * @param v0 The starting value (y1).
     * @param v1 The ending value (y2).
     * @param t The interpolation factor, typically between 0 and 1.
     * @return The interpolated value.
     */
    public static double lerp(double v0, double v1, double t) {
        return v0 + t * (v1 - v0);
    }

    public static Pose lerp(Pose start, Pose end, double t) {
        double x = lerp(start.getX(), end.getX(), t);
        double y = lerp(start.getY(), end.getY(), t);
        return new Pose(x, y);
    }

    /**
     * Finds the index of the largest value in a circular buffer that is <= target.
     * * @param data      The circular array of sorted values (e.g., timestamps).
     * @param target    The value to search for.
     * @param oldestIdx The physical index of the oldest element.
     * @param size      Number of elements currently in the buffer.
     * @return The physical index of the "floor" value.
     */
    public static int findFloorIndex(long[] data, long target, int oldestIdx, int size) {
        int low = 0;
        int high = size - 1;
        int bestMatchPhysIdx = -1;

        while (low <= high) {
            int midVirtual = (low + high) >>> 1;
            int midPhysical = toPhysicalIndex(midVirtual, oldestIdx, data.length);

            if (data[midPhysical] <= target) {
                // This is a candidate, keep it and look for a higher value to the right
                bestMatchPhysIdx = midPhysical;
                low = midVirtual + 1;
            } else {
                // Too high, look to the left
                high = midVirtual - 1;
            }
        }

        return bestMatchPhysIdx;
    }

    private static int toPhysicalIndex(int virtualIdx, int offset, int capacity) {
        return (offset + virtualIdx) % capacity;
    }

    /**
     * Predicts your future {@link Pose} at the lookahead time, based on your velocity and acceleration.
     *
     * @param pose current pose
     * @param velocity current velocity
     * @param angularVelocity current angular velocity in rad/s
     * @param acceleration current acceleration
     * @param angularAcceleration current angular acceleration in rad/s²
     * @param lookaheadTimeSec the time you want to evaluate the future position
     *
     * @return the future pose at the given lookahead time
     */
    public static Pose predictFuturePose(Pose pose, Vector velocity, double angularVelocity, Vector acceleration, double angularAcceleration, double lookaheadTimeSec) {
        double lookaheadTimeSecSquared = lookaheadTimeSec * lookaheadTimeSec;

        Vector linearDisplacement = velocity.times(lookaheadTimeSec);
        Vector acceleratedDisplacement = acceleration.times(0.5 * lookaheadTimeSecSquared);

        Vector totalDisplacement =  linearDisplacement.plus(acceleratedDisplacement);
        double totalRotation = (angularVelocity * lookaheadTimeSec) + (0.5 * angularAcceleration * lookaheadTimeSecSquared);

        return pose.plus(new Pose(totalDisplacement.getXComponent(), totalDisplacement.getYComponent(), totalRotation));
    }

    public static Pose predictFuturePose(Pose pose, Vector velocity, double lookaheadTimeSec) {
        Vector linearDisplacement = velocity.times(lookaheadTimeSec);
        return pose.plus(new Pose(linearDisplacement.getXComponent(), linearDisplacement.getYComponent(), 0));
    }
}
