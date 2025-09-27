package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class RPSController {
    private final DcMotorEx dcMotorEx;
    private final double ticksPerRev;

    private double rotationsPerSecond = 0;

    public RPSController(DcMotorEx dcMotorEx, double ticksPerRev) {
        this.dcMotorEx = dcMotorEx;
        this.ticksPerRev = ticksPerRev;

        dcMotorEx.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void setRPS(double rotationsPerSecond) {
        this.rotationsPerSecond = rotationsPerSecond;
        dcMotorEx.setVelocity(rotationsPerSecond * ticksPerRev);
    }

    public double getConfiguredRPS() {
        return rotationsPerSecond;
    }

    public double getCurrentRPS() {
        return dcMotorEx.getVelocity() / ticksPerRev;
    }

    public void stopFlywheel(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        dcMotorEx.setVelocity(0);
        dcMotorEx.setZeroPowerBehavior(zeroPowerBehavior);
    }

    public DcMotorEx getDcMotorEx() {
        return dcMotorEx;
    }
}
