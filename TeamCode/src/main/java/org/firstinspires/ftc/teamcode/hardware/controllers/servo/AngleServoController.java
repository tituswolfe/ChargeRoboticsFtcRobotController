package org.firstinspires.ftc.teamcode.hardware.controllers.servo;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.math.Angle;

public class AngleServoController extends ServoController {
    public final Angle offset;
    public final Angle maxPositiveLimit;
    public final Angle minNegativeLimit;

    public AngleServoController(Servo servo, Angle totalRotation, double totalGearRatio, Angle offset, Angle maxPositiveLimit, Angle minNegativeLimit) {
        super(servo, totalRotation, totalGearRatio);
        this.offset = offset;
        this.maxPositiveLimit = maxPositiveLimit;
        this.minNegativeLimit = minNegativeLimit;
    }

    public void setTargetAngle(Angle angle) {
        Angle targetAngle = new Angle(Math.min(
                maxPositiveLimit.getAngle(Angle.AngleSystem.SIGNED),
                Math.max(
                        minNegativeLimit.getAngle(Angle.AngleSystem.SIGNED),
                        angle.getAngle(Angle.AngleSystem.SIGNED)
                )
        ));

        servo.setPosition(targetAngle.minus(offset, Angle.AngleSystem.SIGNED).getAngle(Angle.AngleSystem.SIGNED) * percentagePerOutputRadiant);
    }

    public Angle getTargetAngle() {
        return new Angle(servo.getPosition() / percentagePerOutputRadiant);
    }
}
