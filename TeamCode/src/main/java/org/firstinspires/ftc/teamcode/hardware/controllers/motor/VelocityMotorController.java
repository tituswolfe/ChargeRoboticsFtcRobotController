package org.firstinspires.ftc.teamcode.hardware.controllers.motor;

import com.pedropathing.control.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.util.math.MathUtil;

public class VelocityMotorController extends MotorController{
    private double targetRotationsPerMinute = 0;
    private boolean isEngaged = false;

    public VelocityMotorController(DcMotorEx dcMotorEx, DcMotorSimple.Direction direction, PIDFCoefficients pidfCoefficients, double ticksPerRevolution, double totalGearRatio, double maxPower) {
        super(dcMotorEx, direction, pidfCoefficients, ticksPerRevolution, totalGearRatio, maxPower);
        this.dcMotorEx.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void update() {
        pidfController.updateError(targetRotationsPerMinute - getVelocity());
        pidfController.updateFeedForwardInput(targetRotationsPerMinute);

        this.dcMotorEx.setPower(isEngaged ? MathUtil.clamp(pidfController.run(), -maxPower, maxPower) : 0);
    }

    /**
     * @param rotationsPerMinute
     */
    public void setTargetVelocity(double rotationsPerMinute) {
        this.targetRotationsPerMinute = rotationsPerMinute;
    }

    /**
     * @return target velocity in rotations per minute
     */
    public double getTargetVelocity() {
        return targetRotationsPerMinute;
    }

    /**
     * @return velocity in rotations per minute
     */
    public double getVelocity() {
        return (dcMotorEx.getVelocity() * 60) / ticksPerRevolution;
    }

    public boolean isEngaged() {
        return isEngaged;
    }

    public void setEngaged(boolean engaged) {
        this.isEngaged = engaged;
    }

    public DcMotorEx getDcMotorEx() {
        return dcMotorEx;
    }
}
