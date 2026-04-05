package org.firstinspires.ftc.teamcode.hardware.controllers.motor;

import androidx.core.math.MathUtils;

import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.control.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.util.math.MathUtil;

public class VelocityPIDFMotorController extends PIDFMotorController {
    private double targetVelocity = 0;
    private double error;

    public VelocityPIDFMotorController(DcMotorEx device, String name, PIDFCoefficients pidfCoefficients, double ticksPerRevolution, double totalGearRatio, double maxPower) {
        super(device, name, pidfCoefficients, ticksPerRevolution, totalGearRatio, maxPower);
        device.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void update() {
        // 1. Update sensor data
        velocity = (device.getVelocity() * 60) / ticksPerRevolution;
        currentPosition = device.getCurrentPosition();

        // 2. Calculate the NEW power first
        error = targetVelocity - velocity;
        pidfController.updateError(error);
        pidfController.updateFeedForwardInput(targetVelocity);
        setPowerFromPIDFController(); // This sets the 'power' variable

        // 3. Apply safety and Slew Rate to the NEW power
        double clampedPower = MathUtils.clamp(power, -maxPower, maxPower);
        double slewRatePower = Range.clip(clampedPower, lastPower - SLEW_RATE, lastPower + SLEW_RATE);

        // 4. Hardware Write
        if (!isMotorEngaged) {
            device.setPower(0);
        } else if (clampedPower == 0 && lastPower != 0) {
            device.setPower(0);
        } else if (!MathUtil.isWithinRange(slewRatePower, lastPower, DELTA_FILTERING_THRESHOLD)) {
            device.setPower(slewRatePower);
        }

        lastPower = device.getPower(); // Use actual applied power for next loop
    }

//    @Override
//    public void update() {
//        velocity = (device.getVelocity() * 60) / ticksPerRevolution;
//        currentPosition = device.getCurrentPosition();
//
//        double clampedPower = MathUtils.clamp(power, -maxPower, maxPower);
//        double slewRatePower = Range.clip(clampedPower, lastPower - SLEW_RATE, lastPower + SLEW_RATE);
//        if (isMotorEngaged) {
//            device.setPower(0);
//        }
//        else if (!MathUtil.isWithinRange(slewRatePower, lastPower, DELTA_FILTERING_THRESHOLD)) {
//            device.setPower(slewRatePower);
//        } else if (clampedPower == 0 && lastPower != 0) {
//            device.setPower(0);
//        }
//
//        lastPower = clampedPower;
//
//
//        error = targetVelocity - velocity;
//        pidfController.updateError(error);
//        pidfController.updateFeedForwardInput(targetVelocity);
//
//        setPowerFromPIDFController();
//    }

//    @Override
//    public void update() {
//        super.update();
//
//        error = targetVelocity - velocity;
//        pidfController.updateError(error);
//        pidfController.updateFeedForwardInput(targetVelocity);
//
//        setPowerFromPIDFController();
//    }

    public void setTargetVelocity(double targetVelocityRPM) {
        this.targetVelocity = targetVelocityRPM;
    }

    public double getTargetVelocity() {
        return targetVelocity;
    }

    public double getError() {
        return isMotorEngaged ? error : 0;
    }

    @Override
    public void updateTelemetry(TelemetryManager telemetry) {
        super.updateTelemetry(telemetry);
        telemetry.addData(name + "Target Velocity (RPM)", targetVelocity);
    }
}
