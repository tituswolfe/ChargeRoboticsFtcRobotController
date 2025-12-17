package org.firstinspires.ftc.teamcode.util.math;

import static com.ThermalEquilibrium.homeostasis.Utils.MathUtils.TAU;

public class Angle {
    private final double angleInRadians;

    public enum AngleUnit {
        DEGREES,
        RADIANS
    }

    public enum AngleSystem {
        SIGNED, // -360 or less TO 360 or more
        SIGNED_180_WRAPPED, // -180 to 180
        UNSIGNED_WRAPPED, // 0 to 360 (wrapped, 370 = 10)
    }

    public Angle(double angleRadians) {
        this.angleInRadians = angleRadians;
    }

    public Angle(double angle, boolean isRadians) {
        angleInRadians = isRadians ? angle : Math.toRadians(angle);
    }

    public Angle(double angle, AngleUnit angleUnit) {
        angleInRadians = switch (angleUnit) {
            case DEGREES ->  Math.toRadians(angle);
            case RADIANS -> angle;
        };
    }

    /**
     * @param angleUnit degrees or radians
     * @param angleSystem signed (-180 to 180, two-quadrant) or unsigned (0 to 360, positive or full clockwise circle rotation)
     * @return angle with the specified {@link AngleUnit} and {@link AngleSystem}
     */
    public double getAngle(AngleUnit angleUnit, AngleSystem angleSystem) {
        double requestedAngleInRadians = switch (angleSystem) {
            case SIGNED -> this.angleInRadians;
            case SIGNED_180_WRAPPED -> Math.IEEEremainder(this.angleInRadians, MathUtil.TAU);
            case UNSIGNED_WRAPPED -> normalizeAngleUnsigned(this.angleInRadians);
        };

        return (angleUnit == AngleUnit.DEGREES) ? Math.toDegrees(requestedAngleInRadians) : requestedAngleInRadians;
    }

    private static double normalizeAngleUnsigned(double angleInRadians) {
        double remainder = angleInRadians % TAU;
        double normalized = remainder < 0 ? remainder + TAU : remainder;

        return normalized;
    }

    /**
     * @param angleSystem signed (-180 to 180, two-quadrant) or unsigned (0 to 360, positive or full clockwise circle rotation)
     * @return angle in {@link AngleUnit#RADIANS} with the specified {@link AngleSystem}
     */
    public double getAngle(AngleSystem angleSystem) {
        return getAngle(AngleUnit.RADIANS, angleSystem);
    }

    public Angle minus(Angle angle, AngleSystem angleSystem) {
        return new Angle(getAngle(angleSystem) - angle.getAngle(angleSystem));
    }

    public Angle plus(Angle angle, AngleSystem angleSystem) {
        return new Angle(getAngle(angleSystem) + angle.getAngle(angleSystem));
    }

}
