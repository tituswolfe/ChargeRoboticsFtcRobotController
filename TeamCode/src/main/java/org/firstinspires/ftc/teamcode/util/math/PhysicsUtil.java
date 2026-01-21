package org.firstinspires.ftc.teamcode.util.math;

import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;

public class PhysicsUtil {
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
        //return predictFuturePose(pose, velocity, 0, new Vector(0, 0), 0, lookaheadTimeSec);
    }
}
