package org.firstinspires.ftc.teamcode.hardware.controllers.servo;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.math.Angle;

public class ServoController {
    protected final Servo servo;
    protected final Angle totalRotation;
    protected double totalGearRatio;
    protected double percentagePerOutputDegree;
    protected double percentagePerOutputRadian;

    public ServoController(Servo servo, Servo.Direction direction, Angle totalRotation, double totalGearRatio) {
        this.servo = servo;
        this.servo.setDirection(direction);
        this.totalRotation = totalRotation;
        setTotalGearRatio(totalGearRatio);
    }

    public double getTotalGearRatio() {
        return totalGearRatio;
    }

    public void setTotalGearRatio(double totalGearRatio) {
        this.totalGearRatio = totalGearRatio;
        this.percentagePerOutputDegree = (1.0 / totalRotation.getAngle(Angle.AngleUnit.DEGREES, Angle.AngleNormalization.NONE)) * totalGearRatio;
        this.percentagePerOutputRadian = (1.0 / totalRotation.getAngle(Angle.AngleUnit.RADIANS, Angle.AngleNormalization.NONE)) * totalGearRatio;
    }

    public Servo getServo() {
        return servo;
    }
}
