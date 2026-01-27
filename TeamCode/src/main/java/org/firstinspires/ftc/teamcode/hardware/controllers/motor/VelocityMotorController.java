package org.firstinspires.ftc.teamcode.hardware.controllers.motor;

import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.control.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class VelocityMotorController extends MotorController {
    private double targetVelocity = 0;
    private double error = 0;

    public VelocityMotorController(DcMotorEx device, String name, PIDFCoefficients pidfCoefficients, double ticksPerRevolution, double totalGearRatio, double maxPower) {
        super(device, name, pidfCoefficients, ticksPerRevolution, totalGearRatio, maxPower);
        device.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void update() {
        super.update();

        error = targetVelocity - velocity;
        pidfController.updateError(error);
        pidfController.updateFeedForwardInput(targetVelocity);

        setPowerFromPIDFController();
    }

    public void setTargetVelocity(double targetVelocityRPM) {
        this.targetVelocity = targetVelocityRPM;
    }

    public double getTargetVelocity() {
        return targetVelocity;
    }

    public double getError() {
        return error;
    }

    @Override
    public void addTelemetry(TelemetryManager telemetry) {
        super.addTelemetry(telemetry);
        telemetry.addData("Target Velocity (RPM)", targetVelocity);
    }
}
