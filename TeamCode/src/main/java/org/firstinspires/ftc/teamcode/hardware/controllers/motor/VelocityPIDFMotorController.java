package org.firstinspires.ftc.teamcode.hardware.controllers.motor;

import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.control.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.util.info.MotorInfo;

/**
 * {@link VelocityPIDFMotorController} can be used to control a motor by velocity.
 * This class was made specifically for controlling flywheels.
 *
 * @author Titus Wolfe
 */
public class VelocityPIDFMotorController extends PIDFMotorController {
    private double targetOutputVelocity = 0;
    private double lastError = 0;
    private double error = 0;

    public VelocityPIDFMotorController(DcMotorEx device, String name, MotorInfo motorInfo, double totalGearRatio, double maxPower, PIDFCoefficients pidfCoefficients) {
        super(device, name, motorInfo, totalGearRatio, maxPower, pidfCoefficients);
    }

    @Override
    public void update(long deltaTimeNS) {
        // TODO FIRST CYCLE CHECK SOMEDAY FOR D TERM
        super.update(deltaTimeNS);

        double clampedTargetVelocity = Math.min(targetOutputVelocity, maxOutputSpeedRPM);
        double currentError = clampedTargetVelocity - outputVelocity;

        pidfController.updateError(currentError);
        pidfController.updateFeedForwardInput(targetOutputVelocity);
        targetPower = pidfController.run();

        lastError = this.error;
        this.error = currentError;
    }

    public void setTargetOutputVelocity(double targetVelocityRPM) {
        this.targetOutputVelocity = targetVelocityRPM;
    }

    public double getTargetOutputVelocity() {
        return targetOutputVelocity;
    }

    public double getError() {
        return isMotorEngaged ? error : 0;
    }

    public double getLastError() {
        return lastError;
    }

    @Override
    public void updateTelemetry(TelemetryManager telemetry) {
        super.updateTelemetry(telemetry);
        telemetry.addData(name + " target velocity (RPM)", targetOutputVelocity);
        telemetry.addData(name + " error", error);
    }
}
