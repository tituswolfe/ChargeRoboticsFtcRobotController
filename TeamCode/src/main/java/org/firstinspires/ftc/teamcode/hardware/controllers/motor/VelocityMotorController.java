package org.firstinspires.ftc.teamcode.hardware.controllers.motor;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class VelocityMotorController extends MotorController{
    private double targetRotationsPerMinute = 0;

    public VelocityMotorController(DcMotorEx dcMotorEx, double ticksPerRevolution, double totalGearRatio) {
        super(dcMotorEx, ticksPerRevolution, totalGearRatio);

        this.dcMotorEx.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    // TODO: ADJUST ALL INSTANCES

    /**
     * @param rotationsPerMinute
     */
    public void setTargetVelocity(double rotationsPerMinute) {
        this.targetRotationsPerMinute = rotationsPerMinute;
        dcMotorEx.setVelocity((rotationsPerMinute / 60) * ticksPerRevolution);
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
