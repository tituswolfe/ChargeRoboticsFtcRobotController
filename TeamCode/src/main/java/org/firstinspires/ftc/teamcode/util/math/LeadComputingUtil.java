package org.firstinspires.ftc.teamcode.util.math;

import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;

public class LeadComputingUtil {
    public static Pose getFuturePose(Pose currentPose, Vector velocity, Vector acceleration, double lookaheadTimeSec) {
        double lookaheadTimeSecSquared = Math.pow(lookaheadTimeSec, 2);
        double predictX = (velocity.getXComponent() * lookaheadTimeSec) + (0.5 * acceleration.getXComponent() * lookaheadTimeSecSquared);
        double predictY = (velocity.getYComponent() * lookaheadTimeSec) + (0.5 * acceleration.getYComponent() * lookaheadTimeSecSquared);
        double predictHeading = (velocity.getTheta() * lookaheadTimeSec) + (0.5 * acceleration.getTheta() * lookaheadTimeSecSquared); // TODO: Check theta?

        Pose futurePose = currentPose.getPose().plus(new Pose(predictX, predictY, predictHeading));

        return futurePose;
    }

    // TODO NAME CHANGE?
    public static Pose getRelativeVirtualTarget(Pose target, Vector velocity, double timeOfFlightSec) {
        return target.minus(new Pose(velocity.getXComponent() * timeOfFlightSec, velocity.getYComponent() * timeOfFlightSec));
    }

}
