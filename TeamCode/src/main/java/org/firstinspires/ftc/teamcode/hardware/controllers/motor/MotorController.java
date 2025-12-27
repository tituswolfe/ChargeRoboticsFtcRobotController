package org.firstinspires.ftc.teamcode.hardware.controllers.motor;

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.util.math.MathUtil;

public abstract class MotorController {
    protected final DcMotorEx dcMotorEx;
    protected PIDFController pidfController;
    protected final double ticksPerRevolution;

    private double totalGearRatio;
    protected double ticksPerOutputDegree;
    protected double ticksPerOutputRadian;

    protected double maxPower;

    /**
     * @param dcMotorEx
     * @param ticksPerRevolution encoder ticks per revolution (check motor for info)
     * @param totalGearRatio amount of motor rotations per 1 output rotations -> totalGearRatio (input) : 1 (output)
     */
    public MotorController(DcMotorEx dcMotorEx, DcMotorSimple.Direction direction, PIDFCoefficients pidfCoefficients, double ticksPerRevolution, double totalGearRatio, double maxPower) {
        this.dcMotorEx = dcMotorEx;
        this.dcMotorEx.setDirection(direction);
        pidfController = new PIDFController(pidfCoefficients);
        this.ticksPerRevolution = ticksPerRevolution;
        setTotalGearRatio(totalGearRatio);

        assert maxPower > 0;
        this.maxPower = maxPower;

        this.dcMotorEx.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public abstract void update();

    // TODO
//    public setPowerConstrained() {
//
//    }


    public DcMotorEx getDcMotorEx() {
        return dcMotorEx;
    }

    public double getTicksPerRevolution() {
        return ticksPerRevolution;
    }

    public PIDFController getPidfController() {
        return pidfController;
    }

    public void setPidfController(PIDFController pidfController) {
        this.pidfController = pidfController;
    }

    public double getTotalGearRatio() {
        return totalGearRatio;
    }

    public void setTotalGearRatio(double totalGearRatio) {
        this.totalGearRatio = totalGearRatio;
        this.ticksPerOutputDegree = (ticksPerRevolution *  totalGearRatio) / 360;
        this.ticksPerOutputRadian = (ticksPerRevolution *  totalGearRatio) / MathUtil.TAU;
    }

    public double getMaxPower() {
        return maxPower;
    }

    public void setMaxPower(double maxPower) {
        this.maxPower = maxPower;
    }

    public double getTicksPerOutputDegree() {
        return ticksPerOutputDegree;
    }

    public double getTicksPerOutputRadian() {
        return ticksPerOutputRadian;
    }
}
