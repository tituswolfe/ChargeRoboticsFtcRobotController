package org.firstinspires.ftc.teamcode.hardware.controllers.servo;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.hardware.controllers.HardwareController;
import org.firstinspires.ftc.teamcode.util.math.Angle;

public class ServoController extends HardwareController<Servo> {
    protected final Angle totalRotation;
    protected double totalGearRatio;
    protected double percentagePerOutputDegree;
    protected double percentagePerOutputRadian;

    public ServoController(Servo device, String name, Angle totalRotation) {
        super(device, name);

        this.totalRotation = totalRotation;
        setTotalGearRatio(totalGearRatio);
    }

//    public ServoController(Servo servo, Servo.Direction direction, Angle totalRotation, double totalGearRatio) {
//        this.servo = servo;
//        this.servo.setDirection(direction);
//        this.totalRotation = totalRotation;
//        setTotalGearRatio(totalGearRatio);
//    }

    public double getTotalGearRatio() {
        return totalGearRatio;
    }

    public void setTotalGearRatio(double totalGearRatio) {
        this.totalGearRatio = totalGearRatio;
        this.percentagePerOutputDegree = (1.0 / totalRotation.getAngle(Angle.AngleUnit.DEGREES, Angle.AngleNormalization.NONE)) * totalGearRatio;
        this.percentagePerOutputRadian = (1.0 / totalRotation.getAngle(Angle.AngleUnit.RADIANS, Angle.AngleNormalization.NONE)) * totalGearRatio;
    }

    @Override
    public void update() {

    }
}
