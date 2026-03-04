package org.firstinspires.ftc.teamcode.hardware.controllers.motor;

import androidx.core.math.MathUtils;

import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.hardware.controllers.HardwareController;
import org.firstinspires.ftc.teamcode.util.math.MathUtil;

public abstract class PIDFMotorController extends HardwareController<DcMotorEx> {
    protected final PIDFController pidfController;
    protected final double ticksPerRevolution;

    private double totalGearRatio;
    protected double ticksPerOutputDegree;
    protected double ticksPerOutputRadian;

    protected final double maxPower;

    protected double velocity = 0;
    protected double lastPower = 0;
    protected double power = 0;
    protected boolean isMotorEngaged = true;
    protected double currentPosition = 0;

    protected static double DELTA_FILTERING_THRESHOLD = 0.03;
    protected static double SLEW_RATE = 0.2;

    public PIDFMotorController(DcMotorEx device, String name, PIDFCoefficients pidfCoefficients, double ticksPerRevolution, double totalGearRatio, double maxPower) {
        super(device, name);

        pidfController = new PIDFController(pidfCoefficients);
        this.ticksPerRevolution = ticksPerRevolution;
        setTotalGearRatio(totalGearRatio);

        assert maxPower > 0;
        this.maxPower = maxPower;

        device.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    @Override
    public void update() {
        velocity = (device.getVelocity() * 60) / ticksPerRevolution;
        currentPosition = device.getCurrentPosition();

        double clampedPower = MathUtils.clamp(power, -maxPower, maxPower);
        if (!MathUtil.isWithinRange(clampedPower, lastPower, DELTA_FILTERING_THRESHOLD)) {
            device.setPower(clampedPower);
        } else if (clampedPower == 0 && lastPower != 0) {
            device.setPower(0);
        }

        lastPower = clampedPower;
    }

    public void setPower(double power) {
        this.power = isMotorEngaged ? power : 0;
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
        double ticksPerOutputRevolution = (ticksPerRevolution *  totalGearRatio);
        this.ticksPerOutputDegree = ticksPerOutputRevolution / 360;
        this.ticksPerOutputRadian = ticksPerOutputRevolution / MathUtil.TAU;
    }

    public double getVelocity() {
        return velocity;
    }

    public PIDFController getPidfController() {
        return pidfController;
    }

    public void setDeltaFilteringThreshold(double deltaFilteringThreshold) {
        this.DELTA_FILTERING_THRESHOLD = deltaFilteringThreshold;
    }

    @Override
    public void updateTelemetry(TelemetryManager telemetry) {
        super.updateTelemetry(telemetry);

        telemetry.addData(name + " Velocity (RPM)", velocity);
        telemetry.addData(name + " isMotorEngaged", isMotorEngaged);
    }
}
