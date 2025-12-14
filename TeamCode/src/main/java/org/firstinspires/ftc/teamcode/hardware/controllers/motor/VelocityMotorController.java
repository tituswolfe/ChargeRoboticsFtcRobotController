package org.firstinspires.ftc.teamcode.hardware.controllers.motor;

import com.pedropathing.control.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class VelocityMotorController extends MotorController{
    private double targetRotationsPerMinute = 0;

    public VelocityMotorController(DcMotorEx dcMotorEx, DcMotorSimple.Direction direction, PIDFCoefficients pidfCoefficients, double ticksPerRevolution, double totalGearRatio) {
        super(dcMotorEx, direction, pidfCoefficients, ticksPerRevolution, totalGearRatio);
        this.dcMotorEx.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void update() {
        pidfController.updateError(targetRotationsPerMinute - getVelocity());
        this.dcMotorEx.setPower(pidfController.run());
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

    public void breakMotor(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        dcMotorEx.setVelocity(0);
        dcMotorEx.setZeroPowerBehavior(zeroPowerBehavior);
    }

    public DcMotorEx getDcMotorEx() {
        return dcMotorEx;
    }
}
