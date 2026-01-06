package org.firstinspires.ftc.teamcode.hardware.controllers.servo;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.math.Angle;
import org.firstinspires.ftc.teamcode.util.math.MathUtil;

public class AngleServoController extends ServoController {
    public final Angle offset;
    public final Angle maxLimit;
    public final Angle minLimit; // TODO: RENAME



    public AngleServoController(Servo servo, Servo.Direction direction, Angle totalRotation, double totalGearRatio, Angle offset, Angle maxLimit, Angle minLimit) {
        super(servo, direction, totalRotation, totalGearRatio);
        this.offset = offset;
        this.maxLimit = maxLimit;
        this.minLimit = minLimit;
    }

    public void setTargetAngle(Angle angle) {
        Angle targetAngle = new Angle(MathUtil.clamp(
                angle.getAngle(Angle.AngleSystem.SIGNED),
                minLimit.getAngle(Angle.AngleSystem.SIGNED),
                maxLimit.getAngle(Angle.AngleSystem.SIGNED)
        ));

        servo.setPosition(targetAngle.minus(offset, Angle.AngleSystem.SIGNED).getAngle(Angle.AngleSystem.SIGNED) * percentagePerOutputRadian);
    }

    public Angle getTargetAngle() {
        return new Angle(servo.getPosition() / percentagePerOutputRadian).plus(offset, Angle.AngleSystem.SIGNED);
    }
}
