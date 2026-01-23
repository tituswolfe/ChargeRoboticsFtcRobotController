package org.firstinspires.ftc.teamcode.hardware.controllers.servo;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.math.Angle;
import org.firstinspires.ftc.teamcode.util.math.MathUtil;

public class AngleServoController extends ServoController {
//    public final Angle initialOffset;

    public final Angle minSoftLimit;
    public final Angle maxSoftLimit;

    public AngleServoController(Servo device, String name, Angle totalRotation, Angle minSoftLimit, Angle maxSoftLimit) {
        super(device, name, totalRotation);
        this.minSoftLimit = minSoftLimit;
        this.maxSoftLimit = maxSoftLimit;
    }

//    public AngleServoController(Servo servo, Servo.Direction direction, Angle totalRotation, double totalGearRatio, Angle offset, Angle maxSoftLimit, Angle minSoftLimit) {
//        super(servo, direction, totalRotation, totalGearRatio);
//        this.offset = offset;
//        this.maxSoftLimit = maxSoftLimit;
//        this.minSoftLimit = minSoftLimit;
//    }

    public void setTargetAngle(Angle angle) {
        Angle targetAngle = new Angle(MathUtil.clamp(
                angle.getAngle(Angle.AngleNormalization.NONE),
                minSoftLimit.getAngle(Angle.AngleNormalization.NONE),
                maxSoftLimit.getAngle(Angle.AngleNormalization.NONE)
        ));

        device.setPosition(targetAngle.minus(minSoftLimit, Angle.AngleNormalization.NONE).getAngle(Angle.AngleNormalization.NONE) * percentagePerOutputRadian);
    }

    public Angle getTargetAngle() {
        return new Angle(device.getPosition() / percentagePerOutputRadian).plus(minSoftLimit, Angle.AngleNormalization.NONE);
    }
}
