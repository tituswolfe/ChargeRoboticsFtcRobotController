package org.firstinspires.ftc.teamcode.hardware.controllers.motor;

import androidx.core.math.MathUtils;

import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.hardware.controllers.HardwareController;
import org.firstinspires.ftc.teamcode.util.math.MathUtil;

public abstract class MotorController extends HardwareController<DcMotorEx> {
    protected final PIDFController pidfController;
    protected final double ticksPerRevolution;

    private double totalGearRatio;
    protected double ticksPerOutputDegree;
    protected double ticksPerOutputRadian;

    private final double maxPower;

    protected double velocity = 0;
    protected double power = 0;
    protected boolean isMotorEngaged = true;
    protected double currentPosition = 0;

    private double deltaFilteringThreshold = 0.01;

    public MotorController(DcMotorEx device, String name, PIDFCoefficients pidfCoefficients, double ticksPerRevolution, double totalGearRatio, double maxPower) {
        super(device, name);

        // TODO: PID always at 1:1 ratio

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

        device.setPower(power);
    }

    public void setPower(double power) {
        double clampedPower = MathUtils.clamp(power, maxPower, -maxPower);

        // not withing threshold or power is equal zero
        if (!MathUtil.isWithinRange(power, clampedPower, deltaFilteringThreshold) || clampedPower == 0) {
            this.power = isMotorEngaged ? clampedPower : 0;
        }
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
        this.deltaFilteringThreshold = deltaFilteringThreshold;
    }

    @Override
    public void addTelemetry(TelemetryManager telemetry) {
        super.addTelemetry(telemetry);
        telemetry.addData("Velocity (RPM)", velocity);
        telemetry.addData("isMotorEngaged", isMotorEngaged);
        telemetry.addData("PIDF Coefficients", pidfController.getCoefficients().toString());
        telemetry.addData("Total Gear Ratio", totalGearRatio);
        telemetry.addData("Ticks Per Output Degree", ticksPerOutputDegree);
    }
}
