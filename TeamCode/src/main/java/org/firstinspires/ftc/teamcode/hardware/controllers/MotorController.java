package org.firstinspires.ftc.teamcode.hardware.controllers;

import androidx.core.math.MathUtils;

import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.util.math.MathUtil;

public abstract class MotorController extends HardwareController<DcMotorEx> {
    protected PIDFController pidfController;
    protected final double ticksPerRevolution;

    private double totalGearRatio;
    protected double ticksPerOutputDegree;
    protected double ticksPerOutputRadian;

    private final double maxPower;

    protected double velocityRPM = 0;
    protected double power = 0;
    protected boolean isMotorEngaged = true;

    public MotorController(DcMotorEx device, String name, DcMotorSimple.Direction direction, PIDFCoefficients pidfCoefficients, double ticksPerRevolution, double totalGearRatio, double maxPower) {
        super(device, name);

        device.setDirection(direction);
        pidfController = new PIDFController(pidfCoefficients);
        this.ticksPerRevolution = ticksPerRevolution;
        setTotalGearRatio(totalGearRatio);

        assert maxPower > 0;
        this.maxPower = maxPower;

        device.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    @Override
    public void update(long deltaTime) {
        velocityRPM = (device.getVelocity() * 60) / ticksPerRevolution;
        device.setPower(power);
    }

    public void setPower(double power) {
        this.power = isMotorEngaged ? MathUtils.clamp(power, maxPower, -maxPower) : 0;
    }

    protected void setPowerFromPIDFController() {
        setPower(pidfController.run());
    }

    public boolean isMotorEngaged() {
        return isMotorEngaged;
    }

    public void setMotorEngaged(boolean motorEngaged) {
        isMotorEngaged = motorEngaged;
    }

    public void setTotalGearRatio(double totalGearRatio) {
        this.totalGearRatio = totalGearRatio;
        this.ticksPerOutputDegree = (ticksPerRevolution *  totalGearRatio) / 360;
        this.ticksPerOutputRadian = (ticksPerRevolution *  totalGearRatio) / MathUtil.TAU;
    }

    @Override
    public void addTelemetry(TelemetryManager telemetry) {
        super.addTelemetry(telemetry);
        telemetry.addData("Velocity (RPM)", velocityRPM);
        telemetry.addData("isMotorEngaged", isMotorEngaged);
        telemetry.addData("PIDF Coefficients", pidfController.getCoefficients().toString());
        telemetry.addData("Max Power", maxPower);
        telemetry.addData("Total Gear Ratio", totalGearRatio);
        telemetry.addData("Ticks Per Output Degree", ticksPerOutputDegree);
    }
}
