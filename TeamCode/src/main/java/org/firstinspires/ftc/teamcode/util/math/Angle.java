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

        double requestedAngleInRadians;


        if (angleSystem == AngleSystem.SIGNED) {
            requestedAngleInRadians = this.angleInRadians;
        } else {
            // WRAPPED
            // Normalize to [0, TAU)
            double normalizedAngle = this.angleInRadians % TAU;
            if (normalizedAngle < 0) {
                normalizedAngle += TAU;
            }

            if (angleSystem == AngleSystem.UNSIGNED_WRAPPED) {
                // Case 2: UNSIGNED_WRAPPED [0, 2pi)
                requestedAngleInRadians = normalizedAngle;
            } else if (angleSystem == AngleSystem.SIGNED_180_WRAPPED) {
                // Case 3: SIGNED_180 [-pi, pi).
                // Shift the upper half of [0, 2pi) to the negative range.
                requestedAngleInRadians = (normalizedAngle > Math.PI) ? normalizedAngle - TAU : normalizedAngle;
            } else {
                // Fallback/Error handling if another enum value were added
                requestedAngleInRadians = normalizedAngle;
            }
        }

        // 2. CONVERT UNIT
        return (angleUnit == AngleUnit.DEGREES) ? Math.toDegrees(requestedAngleInRadians) : requestedAngleInRadians;

//        double remainder = angleInRadians % TAU;
//        double unsignedAngle = (remainder < 0) ? remainder + TAU : remainder;
//        double requestedAngle = (angleSystem == AngleSystem.SIGNED_180_WRAPPED && unsignedAngle > Math.PI) ? unsignedAngle - TAU : unsignedAngle;
//
//        return (angleUnit == AngleUnit.DEGREES) ? Math.toDegrees(requestedAngle) : requestedAngle;
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

    // TODO: Angle maths
}
