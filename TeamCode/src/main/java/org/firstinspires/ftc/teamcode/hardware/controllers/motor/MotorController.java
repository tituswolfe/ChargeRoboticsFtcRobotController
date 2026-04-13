package org.firstinspires.ftc.teamcode.hardware.controllers.motor;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.hardware.controllers.HardwareController;
import org.firstinspires.ftc.teamcode.util.info.MotorInfo;
import org.firstinspires.ftc.teamcode.util.math.MathUtil;

import java.util.concurrent.TimeUnit;

@Configurable
public abstract class MotorController extends HardwareController<DcMotorEx> {
    protected final MotorInfo motorInfo;
    protected double totalGearRatio;
    protected double ticksPerOutputRevolution;
    protected double ticksPerOutputDegree;
    protected double ticksPerOutputRadian;

    protected final double maxPower;
    protected final double maxOutputSpeedRPM;

    protected double velocity = 0;
    protected double outputVelocity = 0;
    protected double targetPower = 0;
    protected double lastPower = 0;
    protected double currentPosition = 0;
    protected boolean isMotorEngaged = false;
    protected boolean reversePower = false;

    public static double MAX_DELTA_TIME_FOR_SLEW_RATE = 0.1;
    public static double SLEW_RATE = 2; // power / seconds
    public static double DELTA_FILTERING_THRESHOLD = 0.03;

    public MotorController(DcMotorEx device, String name, MotorInfo motorInfo, double totalGearRatio, double maxPower) {
        super(device, name);
        this.motorInfo = motorInfo;
        setTotalGearRatio(totalGearRatio, motorInfo.ENCODER_RESOLUTION_PPR);

        assert maxPower > 0;
        this.maxPower = maxPower;

        maxOutputSpeedRPM = motorInfo.MAX_SPEED_RPM * totalGearRatio;

        device.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void start() {
        device.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        isMotorEngaged = true;
    }

    public void setTotalGearRatio(double totalGearRatio, double encoderResolutionPPR) {
        this.totalGearRatio = totalGearRatio;
        this.ticksPerOutputRevolution = encoderResolutionPPR * totalGearRatio;
        this.ticksPerOutputDegree = ticksPerOutputRevolution / 360;
        this.ticksPerOutputRadian = ticksPerOutputRevolution / MathUtil.TAU;
    }

    @Override
    public void update(long deltaTimeNS) {
        velocity = device.getVelocity() * (60 / motorInfo.ENCODER_RESOLUTION_PPR);
        outputVelocity = velocity / totalGearRatio;
        currentPosition = device.getCurrentPosition();

        if (!isMotorEngaged) {
            if (lastPower != 0) {
                sendPowerToMotor(0);
            }
            return;
        }

        //double deltaTimeSec = Math.min(deltaTimeNS / 1_000_000_000.0, MAX_DELTA_TIME_FOR_SLEW_RATE);
        double maxChange = SLEW_RATE * 1;

        double slewedPower = Range.clip(targetPower, lastPower - maxChange, lastPower + maxChange);
        double clampedPower = Range.clip(slewedPower, -maxPower, maxPower);
        boolean isWithinDeltaFilteringThreshold = MathUtil.isWithinRange(clampedPower, lastPower, DELTA_FILTERING_THRESHOLD);

        if (!isWithinDeltaFilteringThreshold) {
            sendPowerToMotor(reversePower ? -clampedPower : clampedPower);
        }
    }

    protected void sendPowerToMotor(double power) {
        device.setPower(power);
        lastPower = power;
    }

    public boolean isMotorEngaged() {
        return isMotorEngaged;
    }

    public void setMotorEngaged(boolean motorEngaged) {
        isMotorEngaged = motorEngaged;
    }

    public boolean isReversePower() {
        return reversePower;
    }

    public void setReversePower(boolean reversePower) {
        this.reversePower = reversePower;
    }

    @Override
    public void updateTelemetry(TelemetryManager telemetry) {
        super.updateTelemetry(telemetry);

        telemetry.addData(name + " velocity", outputVelocity);
        telemetry.addData(name + " power", targetPower);
        telemetry.addData(name + " isEngaged", isMotorEngaged);
        telemetry.addData(name + " position", currentPosition);
    }
}
