package org.firstinspires.ftc.teamcode.util.math;

import static com.ThermalEquilibrium.homeostasis.Utils.MathUtils.TAU;

public class Angle {
    public double angleInRadians;

    public enum AngleUnit {
        DEGREES,
        RADIANS
    }

    public enum AngleSystem {
        SIGNED,
        UNSIGNED
    }

    public Angle(double angleRadians) {
        this.angleInRadians = angleRadians;
    }

    public Angle(double angle, boolean isRadians) {
        angleInRadians = isRadians ? angle : Math.toRadians(angle);
    }

    /**
     * @param angleUnit degrees or radians
     * @param angleSystem signed (-180 to 180, two-quadrant) or unsigned (0 to 360, positive or full clockwise circle rotation)
     * @return angle with the specified {@link AngleUnit} and {@link AngleSystem}
     */
    public double getAngle(AngleUnit angleUnit, AngleSystem angleSystem) {
        double remainder = angleInRadians % TAU;
        double unsignedAngle = (remainder < 0) ? remainder + TAU : remainder;
        double requestedAngle = (angleSystem == AngleSystem.SIGNED && unsignedAngle > Math.PI) ? unsignedAngle - TAU : unsignedAngle;

        return (angleUnit == AngleUnit.DEGREES) ? Math.toDegrees(requestedAngle) : requestedAngle;
    }
}
