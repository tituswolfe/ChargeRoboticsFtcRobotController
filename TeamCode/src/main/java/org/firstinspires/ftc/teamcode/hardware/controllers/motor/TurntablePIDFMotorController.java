package org.firstinspires.ftc.teamcode.hardware.controllers.motor;

import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.control.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.util.info.MotorInfo;

public class TurntablePIDFMotorController extends PIDFMotorController {
    private final double minAngleLimit;
    private final double maxAngleLimit;

    private double initialAngle = 0;

    private double heading = 0;
    private double targetHeading = 0;
    private double error = 0;
    private double robotHeadingVelocity = 0;

    public TurntablePIDFMotorController(DcMotorEx device, String name, MotorInfo motorInfo, double totalGearRatio, double maxPower, PIDFCoefficients pidfCoefficients, double minAngleLimit, double maxAngleLimit) {
        super(device, name, motorInfo, totalGearRatio, maxPower, pidfCoefficients);

        this.minAngleLimit = minAngleLimit;
        this.maxAngleLimit = maxAngleLimit;
    }


    @Override
    public void update(long deltaTimeNS) {
        super.update(deltaTimeNS);

        heading = (currentPosition / ticksPerOutputRadian) + initialAngle;
        double clippedTargetHeading = Range.clip(targetHeading, minAngleLimit, maxAngleLimit);

        error = clippedTargetHeading - heading;
        pidfController.updateError(Math.toDegrees(error));
        pidfController.updateFeedForwardInput(Math.toDegrees(robotHeadingVelocity));
        targetPower = pidfController.run();
    }

    public double getTargetHeading() {
        return targetHeading;
    }

    public void setTargetHeading(double targetHeading) {
        this.targetHeading = targetHeading;
    }

    public void updateRobotHeadingVelocity(double robotHeadingVelocity) {
        this.robotHeadingVelocity = robotHeadingVelocity;
    }

    public double getInitialAngle() {
        return initialAngle;
    }

    public void setInitialAngle(double initialAngle) {
        this.initialAngle = initialAngle;
    }

    public double getMinAngleLimit() {
        return minAngleLimit;
    }

    public double getMaxAngleLimit() {
        return maxAngleLimit;
    }

    public double getHeading() {
        return heading;
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
