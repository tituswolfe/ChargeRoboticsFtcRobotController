package org.firstinspires.ftc.teamcode.hardware.controllers.motor;

import com.pedropathing.control.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.util.LogicUtil;
import org.firstinspires.ftc.teamcode.util.math.Angle;
import org.firstinspires.ftc.teamcode.util.math.MathUtil;

public class AngleMotorController extends MotorController {
    private final Angle maxPositiveLimit;
    private final Angle minNegativeLimit;
    private final Angle startAngle;
    private final boolean reversePower;

    private Angle targetHeading = new Angle(0);


    public AngleMotorController(DcMotorEx dcMotorEx, DcMotorSimple.Direction direction, PIDFCoefficients pidfCoefficients, double ticksPerRevolution, double totalGearRatio, Angle maxPositiveLimit, Angle minNegativeLimit, Angle startAngle, boolean reversePower) {
        super(dcMotorEx, direction, pidfCoefficients, ticksPerRevolution, totalGearRatio);

        assert maxPositiveLimit.getAngle(Angle.AngleSystem.SIGNED_180_WRAPPED) > minNegativeLimit.getAngle(Angle.AngleSystem.SIGNED_180_WRAPPED);
        this.maxPositiveLimit = maxPositiveLimit;
        this.minNegativeLimit = minNegativeLimit;

        assert startAngle.getAngle(Angle.AngleSystem.SIGNED_180_WRAPPED) < maxPositiveLimit.getAngle(Angle.AngleSystem.SIGNED_180_WRAPPED);
        assert startAngle.getAngle(Angle.AngleSystem.SIGNED_180_WRAPPED) > minNegativeLimit.getAngle(Angle.AngleSystem.SIGNED_180_WRAPPED);
        this.startAngle = startAngle; // TODO: Offset

        this.reversePower = reversePower;

        this.dcMotorEx.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void update() {
        //pidfController.updateError(getConstrainedTurn(targetHeading.getAngle(Angle.AngleUnit.DEGREES, Angle.AngleSystem.SIGNED_180_WRAPPED),  getHeading().getAngle(Angle.AngleUnit.DEGREES, Angle.AngleSystem.SIGNED_180_WRAPPED)));
        pidfController.updateError(getConstrainedError().getAngle(Angle.AngleSystem.SIGNED));
        pidfController.updateFeedForwardInput(targetHeading.getAngle(Angle.AngleUnit.DEGREES, Angle.AngleSystem.SIGNED_180_WRAPPED));
        double power = pidfController.run();
        this.dcMotorEx.setPower(reversePower ? -power : power);
    }

    public void setTargetHeading(Angle targetHeading) {
        this.targetHeading = targetHeading.minus(startAngle, Angle.AngleSystem.SIGNED_180_WRAPPED);
    }


    public Angle getConstrainedError() {
        Angle clampedTargetHeading = new Angle(LogicUtil.clamp(
                targetHeading.getAngle(Angle.AngleSystem.SIGNED_180_WRAPPED),
                minNegativeLimit.getAngle(Angle.AngleSystem.SIGNED_180_WRAPPED),
                maxPositiveLimit.getAngle(Angle.AngleSystem.SIGNED_180_WRAPPED)
        ));
        Angle shortestError = clampedTargetHeading.minus(getHeading(), Angle.AngleSystem.SIGNED_180_WRAPPED);
        Angle predictedHeading = getHeading().plus(shortestError, Angle.AngleSystem.SIGNED_180_WRAPPED);

        if (predictedHeading.getAngle(Angle.AngleSystem.SIGNED_180_WRAPPED) > maxPositiveLimit.getAngle(Angle.AngleSystem.SIGNED_180_WRAPPED) && shortestError.getAngle(Angle.AngleSystem.SIGNED_180_WRAPPED) > 0) {
            return shortestError.minus(new Angle(MathUtil.TAU), Angle.AngleSystem.SIGNED);
        } else if (predictedHeading.getAngle(Angle.AngleSystem.SIGNED_180_WRAPPED) < minNegativeLimit.getAngle(Angle.AngleSystem.SIGNED_180_WRAPPED) && shortestError.getAngle(Angle.AngleSystem.SIGNED_180_WRAPPED) < 0) {
            return shortestError.plus(new Angle(MathUtil.TAU), Angle.AngleSystem.SIGNED);
        }

        return shortestError;
    }


    public Angle getTargetHeading() {
        return new Angle(dcMotorEx.getTargetPosition() / ticksPerOutputRadian).plus(startAngle, Angle.AngleSystem.SIGNED_180_WRAPPED);
    }

    public Angle getHeading() {
        return new Angle(dcMotorEx.getCurrentPosition() / ticksPerOutputRadian).plus(startAngle, Angle.AngleSystem.SIGNED_180_WRAPPED);
    }
}
