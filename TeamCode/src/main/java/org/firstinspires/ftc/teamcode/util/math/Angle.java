package org.firstinspires.ftc.teamcode.util.math;

import static com.ThermalEquilibrium.homeostasis.Utils.MathUtils.TAU;

public class Angle {
    private final double angleInRadians;

    public enum AngleUnit {
        DEGREES,
        RADIANS
    }

    public enum AngleNormalization {
        NONE,
        BIPOLAR, // -180 to 180
        UNIPOLAR, // 0 to 360 (wrapped, 370 = 10)
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
     * @param angleNormalization signed (-180 to 180, two-quadrant) or unsigned (0 to 360, positive or full clockwise circle rotation)
     * @return angle with the specified {@link AngleUnit} and {@link AngleNormalization}
     */
    public double getAngle(AngleUnit angleUnit, AngleNormalization angleNormalization) {
        double requestedAngleInRadians = switch (angleNormalization) {
            case NONE -> this.angleInRadians;
            case BIPOLAR -> Math.IEEEremainder(this.angleInRadians, MathUtil.TAU);
            case UNIPOLAR -> normalizeAngleUnsigned(this.angleInRadians);
        };

        return (angleUnit == AngleUnit.DEGREES) ? Math.toDegrees(requestedAngleInRadians) : requestedAngleInRadians;
    }

    private static double normalizeAngleUnsigned(double angleInRadians) {
        double remainder = angleInRadians % TAU;
        double normalized = remainder < 0 ? remainder + TAU : remainder;

        return normalized;
    }

    /**
     * @param angleNormalization signed (-180 to 180, two-quadrant) or unsigned (0 to 360, positive or full clockwise circle rotation)
     * @return angle in {@link AngleUnit#RADIANS} with the specified {@link AngleNormalization}
     */
    public double getAngle(AngleNormalization angleNormalization) {
        return getAngle(AngleUnit.RADIANS, angleNormalization);
    }

    public Angle minus(Angle angle, AngleNormalization angleNormalization) {
        return new Angle(getAngle(angleNormalization) - angle.getAngle(angleNormalization));
    }

    public Angle plus(Angle angle, AngleNormalization angleNormalization) {
        return new Angle(getAngle(angleNormalization) + angle.getAngle(angleNormalization));
    }

    public Angle times(double scalar, AngleNormalization angleNormalization) {
        return new Angle(getAngle(angleNormalization) * scalar);
    }
//
//    public Angle times(Angle angle, AngleNormalization) {
//
//    }

}
