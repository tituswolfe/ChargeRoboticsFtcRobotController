package org.firstinspires.ftc.teamcode.hardware.controllers.motor;

import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.control.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class VelocityMotorController extends MotorController {
    private double targetVelocity = 0;

    public VelocityMotorController(DcMotorEx device, String name, PIDFCoefficients pidfCoefficients, double ticksPerRevolution, double totalGearRatio, double maxPower) {
        super(device, name, pidfCoefficients, ticksPerRevolution, totalGearRatio, maxPower);
        device.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setTargetVelocity(double targetVelocityRPM) {
        this.targetVelocity = targetVelocityRPM;
    }

    @Override
    public void update() {
        pidfController.updateError(targetVelocity - velocity);
        pidfController.updateFeedForwardInput(targetVelocity);

        setPowerFromPIDFController();
    }

    public double getTargetVelocity() {
        return targetVelocity;
    }

    @Override
    public void addTelemetry(TelemetryManager telemetry) {
        super.addTelemetry(telemetry);
        telemetry.addData("Target Velocity (RPM)", targetVelocity);
    }
}
