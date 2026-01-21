package org.firstinspires.ftc.teamcode.hardware.controllers;

import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.control.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class VelocityMotorController extends MotorController {
    private double targetVelocityRPM = 0;

    public VelocityMotorController(DcMotorEx device, String name, DcMotorSimple.Direction direction, PIDFCoefficients pidfCoefficients, double ticksPerRevolution, double totalGearRatio, double maxPower) {
        super(device, name, direction, pidfCoefficients, ticksPerRevolution, totalGearRatio, maxPower);
        device.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setTargetVelocity(double targetVelocityRPM) {
        this.targetVelocityRPM = targetVelocityRPM;
    }

    @Override
    public void update(long deltaTime) {
        super.update(deltaTime);

        pidfController.updateError(targetVelocityRPM - velocityRPM);
        pidfController.updateFeedForwardInput(targetVelocityRPM);

        setPowerFromPIDFController();
    }

    @Override
    public void addTelemetry(TelemetryManager telemetry) {
        super.addTelemetry(telemetry);
        telemetry.addData("Target Velocity (RPM)", targetVelocityRPM);
    }
}
