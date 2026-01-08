package org.firstinspires.ftc.teamcode.hardware.controllers.motor;

import com.pedropathing.control.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.util.math.Angle;
import org.firstinspires.ftc.teamcode.util.math.MathUtil;

public class TurntableMotorController extends MotorController {
    private final Angle maxPositiveLimit;
    private final Angle minNegativeLimit;
    private final Angle startAngle;
    private final boolean reversePower;

    private Angle targetHeading = new Angle(0);

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

    public Angle getConstrainedError() {
        double currentPos = getHeading().getAngle(Angle.AngleSystem.SIGNED_180_WRAPPED);

        double clampedTarget = MathUtil.clamp(
                targetHeading.getAngle(Angle.AngleSystem.SIGNED_180_WRAPPED),
                minNegativeLimit.getAngle(Angle.AngleSystem.SIGNED_180_WRAPPED),
                maxPositiveLimit.getAngle(Angle.AngleSystem.SIGNED_180_WRAPPED)
        );

        double linearError = clampedTarget - currentPos;

        return new Angle(linearError);
    }


    public Angle getTargetHeading() {
        return targetHeading;
    }

    public Angle getHeading() {
        return new Angle(dcMotorEx.getCurrentPosition() / ticksPerOutputRadian).plus(startAngle, Angle.AngleSystem.SIGNED_180_WRAPPED);
    }
}
