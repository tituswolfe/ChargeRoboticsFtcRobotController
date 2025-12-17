package org.firstinspires.ftc.teamcode.hardware.controllers.servo;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.math.Angle;

public class ServoController {
    protected final Servo servo;
    protected final Angle totalRotation;
    protected double totalGearRatio;
    protected double percentagePerOutputDegree;
    protected double percentagePerOutputRadiant;

    public ServoController(Servo servo, Angle totalRotation, double totalGearRatio) {
        this.servo = servo;
        this.totalRotation = totalRotation;
        setTotalGearRatio(totalGearRatio);
    }

    public double getTotalGearRatio() {
        return totalGearRatio;
    }

    public void setTotalGearRatio(double totalGearRatio) {
        this.totalGearRatio = totalGearRatio;
        this.percentagePerOutputDegree = 1 / (totalRotation.getAngle(Angle.AngleUnit.DEGREES, Angle.AngleSystem.UNSIGNED_WRAPPED) *  totalGearRatio);
        this.percentagePerOutputRadiant = 1 / (totalRotation.getAngle(Angle.AngleUnit.RADIANS, Angle.AngleSystem.UNSIGNED_WRAPPED) * totalGearRatio);
    }

    public Servo getServo() {
        return servo;
    }
}
