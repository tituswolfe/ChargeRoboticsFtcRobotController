package org.firstinspires.ftc.teamcode.hardware.controllers.motor;

import com.pedropathing.control.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.util.math.Angle;
import org.firstinspires.ftc.teamcode.util.math.MathUtil;

/**
 * Allows you to control a motor by output angle (after gear ratio, and ticks
 */
public class TurntableMotorController extends MotorController {
    private final Angle maxPositiveLimit;
    private final Angle minNegativeLimit;
    private final Angle startAngle;
    private final boolean reversePower;

    private Angle targetHeading = new Angle(0);
    private final double deadzoneDeg;

    /**
     * @param dcMotorEx
     * @param direction
     * @param pidfCoefficients
     * @param ticksPerRevolution encoder ticks per revolution (check motor for info)
     * @param totalGearRatio     amount of motor rotations per 1 output rotations -> totalGearRatio (input) : 1 (output)
     * @param maxPower
     */
    public TurntableMotorController(DcMotorEx dcMotorEx, DcMotorSimple.Direction direction, PIDFCoefficients pidfCoefficients, double ticksPerRevolution, double totalGearRatio, double maxPower, Angle maxPositiveLimit, Angle minNegativeLimit, Angle startAngle, boolean reversePower, double deadzoneDeg) {
        super(dcMotorEx, direction, pidfCoefficients, ticksPerRevolution, totalGearRatio, maxPower);
        this.deadzoneDeg = deadzoneDeg;

        assert maxPositiveLimit.getAngle(Angle.AngleSystem.SIGNED_180_WRAPPED) > minNegativeLimit.getAngle(Angle.AngleSystem.SIGNED_180_WRAPPED);
        this.maxPositiveLimit = maxPositiveLimit;
        this.minNegativeLimit = minNegativeLimit;

        assert startAngle.getAngle(Angle.AngleSystem.SIGNED_180_WRAPPED) < maxPositiveLimit.getAngle(Angle.AngleSystem.SIGNED_180_WRAPPED);
        assert startAngle.getAngle(Angle.AngleSystem.SIGNED_180_WRAPPED) > minNegativeLimit.getAngle(Angle.AngleSystem.SIGNED_180_WRAPPED);
        this.startAngle = startAngle;

        this.reversePower = reversePower;

        this.dcMotorEx.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void update() {
        pidfController.updateError(getConstrainedError().getAngle(Angle.AngleUnit.DEGREES, Angle.AngleSystem.SIGNED));
        pidfController.updateFeedForwardInput(targetHeading.getAngle(Angle.AngleUnit.DEGREES, Angle.AngleSystem.SIGNED_180_WRAPPED));

        double power = pidfController.run();
        this.dcMotorEx.setPower(reversePower ? -power : power);
    }

    public void setTargetHeading(Angle targetHeading) {
        this.targetHeading = targetHeading;
    }


//    public Angle getConstrainedError() {
//        Angle clampedTargetHeading = new Angle(LogicUtil.clamp(
//                targetHeading.getAngle(Angle.AngleSystem.SIGNED_180_WRAPPED),
//                minNegativeLimit.getAngle(Angle.AngleSystem.SIGNED_180_WRAPPED),
//                maxPositiveLimit.getAngle(Angle.AngleSystem.SIGNED_180_WRAPPED)
//        ));
//        Angle shortestError = clampedTargetHeading.minus(getHeading(), Angle.AngleSystem.SIGNED_180_WRAPPED);
//        Angle predictedHeading = new Angle(getHeading().getAngle(Angle.AngleSystem.SIGNED_180_WRAPPED) + shortestError.getAngle(Angle.AngleSystem.SIGNED_180_WRAPPED));
//
//        if (predictedHeading.getAngle(Angle.AngleSystem.SIGNED) > maxPositiveLimit.getAngle(Angle.AngleSystem.SIGNED_180_WRAPPED) && shortestError.getAngle(Angle.AngleSystem.SIGNED) > 0) {
//            return shortestError.minus(new Angle(MathUtil.TAU), Angle.AngleSystem.SIGNED);
//        } else if (predictedHeading.getAngle(Angle.AngleSystem.SIGNED) < minNegativeLimit.getAngle(Angle.AngleSystem.SIGNED_180_WRAPPED) && shortestError.getAngle(Angle.AngleSystem.SIGNED) < 0) {
//            return shortestError.plus(new Angle(MathUtil.TAU), Angle.AngleSystem.SIGNED);
//        }
//
//        return shortestError;
//    }

//    public Angle getConstrainedError() {
//        double currentPos = getHeading().getAngle(Angle.AngleSystem.SIGNED_180_WRAPPED);
//        double maxLimit = maxPositiveLimit.getAngle(Angle.AngleSystem.SIGNED_180_WRAPPED);
//        double minLimit = minNegativeLimit.getAngle(Angle.AngleSystem.SIGNED_180_WRAPPED);
//
//        Angle clampedTargetHeading = new Angle(MathUtil.clamp(
//                targetHeading.getAngle(Angle.AngleSystem.SIGNED_180_WRAPPED),
//                minLimit,
//                maxLimit
//        ));
//
//        Angle shortestError = clampedTargetHeading.minus(getHeading(), Angle.AngleSystem.SIGNED_180_WRAPPED);
//        double errorVal = shortestError.getAngle(Angle.AngleSystem.SIGNED_180_WRAPPED);
//
//        double predictedPosition = currentPos + errorVal;
//
//        // 5. Check if the shortest path would cross the "forbidden" hardstop zone
//        // If it does, we force the "long way around"
//        if (predictedPosition > maxLimit + Math.toRadians(deadzoneDeg) && errorVal > 0) {
//            // Shortest path is Clockwise but hits max limit; go Counter-Clockwise instead
//            return new Angle(errorVal - 360, Angle.AngleUnit.DEGREES);
//        } else if (predictedPosition < minLimit - Math.toRadians(deadzoneDeg) && errorVal < 0) {
//            // Shortest path is Counter-Clockwise but hits min limit; go Clockwise instead
//            return new Angle(errorVal + 360, Angle.AngleUnit.DEGREES);
//        }
//
//        // 6. Final safety: If the shortest path is extremely close to 180/-180,
//        // zero it out until it settles.
//        if (Math.abs(Math.abs(errorVal) - 180.0) < deadzoneDeg) {
//            return new Angle(0, Angle.AngleUnit.DEGREES);
//        }
//
//        return shortestError;
//    }

//    public Angle getConstrainedError() {
//        double currentPos = getHeading().getAngle(Angle.AngleSystem.SIGNED_180_WRAPPED);
//        double maxLimit = maxPositiveLimit.getAngle(Angle.AngleSystem.SIGNED_180_WRAPPED);
//        double minLimit = minNegativeLimit.getAngle(Angle.AngleSystem.SIGNED_180_WRAPPED);
//
//        Angle clampedTargetHeading = new Angle(MathUtil.clamp(
//                targetHeading.getAngle(Angle.AngleSystem.SIGNED_180_WRAPPED),
//                minLimit,
//                maxLimit
//        ));
//
//        Angle shortestError = clampedTargetHeading.minus(getHeading(), Angle.AngleSystem.SIGNED);
//        double errorVal = shortestError.getAngle(Angle.AngleSystem.SIGNED_180_WRAPPED);
//
//        double predictedPosition = currentPos + errorVal;
//
//        // 5. Check if the shortest path would cross the "forbidden" hardstop zone
//        // If it does, we force the "long way around"
//        if (predictedPosition > maxLimit && errorVal > 0) {
//            // Shortest path is Clockwise but hits max limit; go Counter-Clockwise instead
//            return new Angle(errorVal).minus(new Angle(360, false), Angle.AngleSystem.SIGNED_180_WRAPPED);
//        } else if (predictedPosition < minLimit && errorVal < 0) {
//            // Shortest path is Counter-Clockwise but hits min limit; go Clockwise instead
//            return new Angle(errorVal).minus(new Angle(360, false), Angle.AngleSystem.SIGNED_180_WRAPPED);
//        }
//
//        return shortestError;
//    }

    public Angle getConstrainedError() {
        double currentPos = getHeading().getAngle(Angle.AngleSystem.SIGNED_180_WRAPPED);

        // 1. Clamp the target to your physical constraints
        double clampedTarget = MathUtil.clamp(
                targetHeading.getAngle(Angle.AngleSystem.SIGNED_180_WRAPPED),
                minNegativeLimit.getAngle(Angle.AngleSystem.SIGNED_180_WRAPPED),
                maxPositiveLimit.getAngle(Angle.AngleSystem.SIGNED_180_WRAPPED)
        );

        // 2. Simple linear error (no wrapping)
        // Because both current and target are clamped/wrapped to the same -180 to 180 range,
        // the difference will not exceed 360 degrees.
        double linearError = clampedTarget - currentPos;

        // 3. Return as a standard Angle
        return new Angle(linearError);
    }


    public Angle getTargetHeading() {
        return targetHeading;
    }

    public Angle getHeading() {
        return new Angle(dcMotorEx.getCurrentPosition() / ticksPerOutputRadian).plus(startAngle, Angle.AngleSystem.SIGNED_180_WRAPPED);
    }
}
