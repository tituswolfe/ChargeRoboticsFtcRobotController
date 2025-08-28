package org.firstinspires.ftc.teamcode.hardware.odometry;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.math.Constants;

/**
 * {@link Odometry} is an abstraction for other using different types of odometry.
 * This allows other systems such as {@link org.firstinspires.ftc.teamcode.hardware.drivetrain.DriveTrain} to call the same methods no matter the odometry system.
 * {@link Odometry} also contains static methods to normalize and unnormalize headings. Standard inch/sec.
 */
public abstract class Odometry {
    public abstract double getXPos(DistanceUnit distanceUnit);
    public abstract double getYPos(DistanceUnit distanceUnit);
    public abstract double getHeading(AngleUnit angleUnit, AngleSystem angleSystem);
    public abstract double getXVel(DistanceUnit distanceUnit);
    public abstract double getYVel(DistanceUnit distanceUnit);
    // dist/sec
    public abstract double getHeadingVel(AngleUnit angleUnit);
    public abstract Pose2D getPose2D();
    public abstract boolean setPose2D(Pose2D pose2D);
    public abstract boolean update();
    public abstract boolean setPosition(Pose2D pose2D);
    public abstract boolean resetPos();
    public abstract boolean resetHeading();
    public abstract boolean resetPosAndHeading();

    /**
     * {@link AngleSystem} specifies weather an angle is defined from 0 to 360 or -180 to 180
     */
    public enum AngleSystem {
        SIGNED,
        UNSIGNED
    }

    public static double convertAngleSystem(double angle, AngleUnit angleUnit, AngleSystem newAngleSystem) {
        if (angleUnit == AngleUnit.RADIANS) {
            if (newAngleSystem == AngleSystem.SIGNED) {
                return AngleUnit.normalizeRadians(angle);
            } else {
                return (angle < 0) ? angle + Constants.tau : angle;
            }
        } else {
            if (newAngleSystem == AngleSystem.SIGNED) {
                return AngleUnit.normalizeDegrees(angle);
            } else {
                return (angle < 0) ? angle + 360 : angle;
            }
        }
    }
}
