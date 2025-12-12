package org.firstinspires.ftc.teamcode.hardware.controllers.servo;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.math.Angle;

public class AngleServoController extends ServoController {
    public final Angle offset; // TODO: Make sure signed and unsigned logic works for this
    public final Angle maxPositiveLimit;
    public final Angle minNegativeLimit;

    public AngleServoController(Servo servo, Angle maxRotation, Angle offset, Angle maxPositiveLimit, Angle minNegativeLimit) {
        super(servo, maxRotation);
        this.offset = offset;
        this.maxPositiveLimit = maxPositiveLimit;
        this.minNegativeLimit = minNegativeLimit;
    }

    public void setTargetAngle(Angle angle) {
        double targetAngle = Math.min(
                Math.max(angle.getAngle(Angle.AngleUnit.RADIANS, Angle.AngleSystem.SIGNED) - offset.getAngle(Angle.AngleUnit.RADIANS, Angle.AngleSystem.SIGNED), minNegativeLimit.angleInRadians),
                maxPositiveLimit.angleInRadians
        );

        servo.setPosition(targetAngle / maxRotation.getAngle(Angle.AngleUnit.RADIANS, Angle.AngleSystem.UNSIGNED));
    }

    public Angle getTargetAngle() {
        return new Angle(servo.getPosition() / (maxRotation.getAngle(Angle.AngleUnit.RADIANS, Angle.AngleSystem.UNSIGNED)) + offset.getAngle(Angle.AngleUnit.RADIANS, Angle.AngleSystem.UNSIGNED));
    }
}
