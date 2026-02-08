package org.firstinspires.ftc.teamcode.hardware.controllers.motor;

import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.control.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.util.math.Angle;
import org.firstinspires.ftc.teamcode.util.math.MathUtil;

public class TurntableMotorController extends MotorController {
    private final Angle minSoftLimit;
    private final Angle maxSoftLimit;

    private final boolean reversePower;

    private Angle initialAngle;

    private Angle targetHeading = new Angle(0);
    private Angle currentHeading = new Angle(0);

    private double error = 0;

    public TurntableMotorController(DcMotorEx device, String name, PIDFCoefficients pidfCoefficients, double ticksPerRevolution, double totalGearRatio, double maxPower, Angle minSoftLimit, Angle maxSoftLimit, boolean reversePower, Angle initialAngle) {
        super(device, name, pidfCoefficients, ticksPerRevolution, totalGearRatio, maxPower);

        this.minSoftLimit = minSoftLimit;
        this.maxSoftLimit = maxSoftLimit;

        this.reversePower = reversePower;

        this.initialAngle = initialAngle;

        device.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void update() {
        super.update();
        currentHeading = new Angle(currentPosition / ticksPerOutputRadian).plus(initialAngle, Angle.AngleNormalization.NONE);

        double clampedTarget = MathUtil.clamp(
                targetHeading.getAngle(Angle.AngleNormalization.BIPOLAR),
                minSoftLimit.getAngle(Angle.AngleNormalization.NONE),
                maxSoftLimit.getAngle(Angle.AngleNormalization.NONE)
        );

        double linearError = clampedTarget - currentHeading.getAngle(Angle.AngleNormalization.BIPOLAR);
        // 130 - 1
        error = linearError;

        pidfController.updateError(Math.toDegrees(linearError));

        setPowerFromPIDFController();
    }

    @Override
    public void setPower(double power) {
        super.setPower(reversePower ? -power : power);
    }

    public void setTargetHeading(Angle targetHeading) {
        this.targetHeading = targetHeading;
    }


    public Angle getTargetHeading() {
        return targetHeading;
    }

    public Angle getHeading() {
        return currentHeading;
    }

    public void setInitialAngle(Angle initialAngle) {
        this.initialAngle = initialAngle;
    }

    public Angle getInitialAngle() {
        return initialAngle;
    }

    public double getError() {
        return error;
    }

    @Override
    public void addTelemetry(TelemetryManager telemetry) {
        super.addTelemetry(telemetry);
        telemetry.addData(name + "Relative Heading", currentHeading.getAngle(Angle.AngleUnit.DEGREES, Angle.AngleNormalization.NONE));
    }
}
