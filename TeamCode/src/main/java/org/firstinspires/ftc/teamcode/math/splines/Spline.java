package org.firstinspires.ftc.teamcode.math.splines;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

/**
 * {@link Spline} is an abstraction for defining and using paramtic curves, specificly for pathfollowing.
 *
 * See also
 * <a href="https://en.wikipedia.org/wiki/Spline_(mathematics)">Spline (mathematics) - Wikipedia</a>
 */
public abstract class Spline {
    private double length;

    abstract void calculateCoefficients(Pose2D p0, Pose2D p1, double scaleFactor);
    abstract void calculateLength(int resolution);

    abstract Pose2D evaluateAt(double length);
    abstract Pose2D evaluateAtNormalized(double t);

    public double getLength() {
        return length;
    };
    public void setLength(double length) {
        this.length = length;
    };
}
