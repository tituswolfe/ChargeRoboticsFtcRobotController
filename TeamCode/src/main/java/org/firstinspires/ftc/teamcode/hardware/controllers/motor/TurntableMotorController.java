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

    /**
     * @param dcMotorEx
     * @param direction
     * @param pidfCoefficients
     * @param ticksPerRevolution encoder ticks per revolution (check motor for info)
     * @param totalGearRatio     amount of motor rotations per 1 output rotations -> totalGearRatio (input) : 1 (output)
     * @param maxPower
     */
    public TurntableMotorController(DcMotorEx dcMotorEx, DcMotorSimple.Direction direction, PIDFCoefficients pidfCoefficients, double ticksPerRevolution, double totalGearRatio, double maxPower, Angle maxPositiveLimit, Angle minNegativeLimit, Angle startAngle, boolean reversePower) {
        super(dcMotorEx, direction, pidfCoefficients, ticksPerRevolution, totalGearRatio, maxPower);

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

    public Angle getConstrainedError() {
        // 1. Get current heading in the same system as your limits (-180 to 180)
        double currentPos = getHeading().getAngle(Angle.AngleSystem.SIGNED_180_WRAPPED);
        double maxLimit = maxPositiveLimit.getAngle(Angle.AngleSystem.SIGNED_180_WRAPPED);
        double minLimit = minNegativeLimit.getAngle(Angle.AngleSystem.SIGNED_180_WRAPPED);

        // 2. Clamp the target to ensure it's within the hardstops
        Angle clampedTargetHeading = new Angle(MathUtil.clamp(
                targetHeading.getAngle(Angle.AngleSystem.SIGNED_180_WRAPPED),
                minLimit,
                maxLimit
        ));

        // 3. Calculate the standard shortest path error
        Angle shortestError = clampedTargetHeading.minus(getHeading(), Angle.AngleSystem.SIGNED_180_WRAPPED);
        double errorVal = shortestError.getAngle(Angle.AngleSystem.SIGNED_180_WRAPPED);

        // 4. Predict the result of this move
        double predictedPosition = currentPos + errorVal;

        // 5. Check if the shortest path would cross the "forbidden" hardstop zone
        // If it does, we force the "long way around"
        if (predictedPosition > maxLimit && errorVal > 0) {
            // Shortest path is Clockwise but hits max limit; go Counter-Clockwise instead
            return new Angle(errorVal - 360, Angle.AngleUnit.DEGREES);
        } else if (predictedPosition < minLimit && errorVal < 0) {
            // Shortest path is Counter-Clockwise but hits min limit; go Clockwise instead
            return new Angle(errorVal + 360, Angle.AngleUnit.DEGREES);
        }

        return shortestError;
    }


    public Angle getTargetHeading() {
        return targetHeading;
    }

    public Angle getHeading() {
        return new Angle(dcMotorEx.getCurrentPosition() / ticksPerOutputRadian).plus(startAngle, Angle.AngleSystem.SIGNED_180_WRAPPED);
    }
}
