package org.firstinspires.ftc.teamcode.hardware.controllers.motor;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.util.math.MathUtil;

public abstract class MotorController {
    protected final DcMotorEx dcMotorEx;
    protected final double ticksPerRevolution;
    protected double ticksPerOutputDegree;
    protected double ticksPerOutputRadian;
    private double totalGearRatio;

    /**
     * @param dcMotorEx
     * @param ticksPerRevolution encoder ticks per revolution (check motor for info)
     * @param totalGearRatio amount of motor rotations per 1 output rotations -> totalGearRatio (input) : 1 (output)
     */
    public MotorController(DcMotorEx dcMotorEx, DcMotorSimple.Direction direction, double ticksPerRevolution, double totalGearRatio) {
        this.dcMotorEx = dcMotorEx;
        this.dcMotorEx.setDirection(direction);
        this.ticksPerRevolution = ticksPerRevolution;
        this.ticksPerOutputDegree = (ticksPerRevolution *  totalGearRatio) / 360;
        this.ticksPerOutputRadian = (ticksPerRevolution *  totalGearRatio) / MathUtil.TAU;
        this.totalGearRatio = totalGearRatio;
    }

    public DcMotorEx getDcMotorEx() {
        return dcMotorEx;
    }

    public double getTicksPerRevolution() {
        return ticksPerRevolution;
    }

    public double getTicksPerOutputDegree() {
        return ticksPerOutputDegree;
    }

    public double getTotalGearRatio() {
        return totalGearRatio;
    }

    public void setTotalGearRatio(double totalGearRatio) {
        this.totalGearRatio = totalGearRatio;
        this.ticksPerOutputDegree = (ticksPerRevolution *  totalGearRatio) / 360;
        this.ticksPerOutputRadian = (ticksPerRevolution *  totalGearRatio) / MathUtil.TAU;
    }
}
