package org.firstinspires.ftc.teamcode.hardware.controllers.servo;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.LogicUtil;
import org.firstinspires.ftc.teamcode.util.math.Angle;

public class AngleServoController extends ServoController {
    public final Angle offset;
    public final Angle maxPositiveLimit;
    public final Angle minNegativeLimit; // TODO: RENAME

    public AngleServoController(Servo servo, Angle totalRotation, double totalGearRatio, Angle offset, Angle maxPositiveLimit, Angle minNegativeLimit) {
        super(servo, totalRotation, totalGearRatio);
        this.offset = offset;
        this.maxPositiveLimit = maxPositiveLimit;
        this.minNegativeLimit = minNegativeLimit;
        servo.setDirection(Servo.Direction.REVERSE);
    }

    public void setTargetAngle(Angle angle) {
        Angle targetAngle = new Angle(LogicUtil.clamp(
                angle.getAngle(Angle.AngleSystem.SIGNED),
                minNegativeLimit.getAngle(Angle.AngleSystem.SIGNED),
                maxPositiveLimit.getAngle(Angle.AngleSystem.SIGNED)
        ));

        servo.setPosition(targetAngle.minus(offset, Angle.AngleSystem.SIGNED).getAngle(Angle.AngleSystem.SIGNED) * percentagePerOutputRadian);
    }

    public Angle getTargetAngle() {
        return new Angle(servo.getPosition() / percentagePerOutputRadian).plus(offset, Angle.AngleSystem.SIGNED);
    }
}
