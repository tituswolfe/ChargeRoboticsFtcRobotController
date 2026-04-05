package org.firstinspires.ftc.teamcode.hardware.controllers.motor;

import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.control.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

public class TurntablePIDFMotorController extends PIDFMotorController {
    private final double minAngleLimit;
    private final double maxAngleLimit;

    private final boolean reversePower;

    private double initialAngle = 0;

    private double heading = 0;
    private double targetHeading = 0;
    private double error = 0;
    private double driveTrainHeadingVelocity = 0;

    public TurntablePIDFMotorController(DcMotorEx device, String name, PIDFCoefficients pidfCoefficients, double ticksPerRevolution, double totalGearRatio, double maxPower, double minAngleLimit, double maxAngleLimit, boolean reversePower, double initialAngle) {
        super(device, name, pidfCoefficients, ticksPerRevolution, totalGearRatio, maxPower);

        this.minAngleLimit = minAngleLimit;
        this.maxAngleLimit = maxAngleLimit;

        this.reversePower = reversePower;

        this.initialAngle = initialAngle;

        device.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void update() {
        super.update();

        heading = (currentPosition / ticksPerOutputRadian) + initialAngle;
        double clippedTargetHeading = Range.clip(targetHeading, minAngleLimit, maxAngleLimit);

        error = clippedTargetHeading - heading; // linear error
        pidfController.updateError(Math.toDegrees(error));
        pidfController.updateFeedForwardInput(Math.toDegrees(driveTrainHeadingVelocity));

        setPowerFromPIDFController();
    }

    @Override
    public void setPower(double power) {
        super.setPower(reversePower ? -power : power);
    }

    public void setTargetHeading(double targetHeading) {
        this.targetHeading = targetHeading;
    }

    public void setDriveTrainHeadingVelocity(double driveTrainHeadingVelocity) {
        this.driveTrainHeadingVelocity = driveTrainHeadingVelocity;
    }


    public double getMinAngleLimit() {
        return minAngleLimit;
    }

    public double getMaxAngleLimit() {
        return maxAngleLimit;
    }

    public boolean isReversePower() {
        return reversePower;
    }

    public double getInitialAngle() {
        return initialAngle;
    }

    public double getHeading() {
        return heading;
    }

    public double getTargetHeading() {
        return targetHeading;
    }

    public double getError() {
        return error;
    }

    @Override
    public void updateTelemetry(TelemetryManager telemetry) {
        super.updateTelemetry(telemetry);
        telemetry.addData(name + " Heading", Math.toDegrees(heading));
        telemetry.addData(name + " Heading Error", Math.toDegrees(error));
    }
}
