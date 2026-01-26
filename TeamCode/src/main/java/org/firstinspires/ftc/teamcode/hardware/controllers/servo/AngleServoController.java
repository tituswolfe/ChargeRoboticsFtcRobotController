package org.firstinspires.ftc.teamcode.hardware.controllers.servo;

import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.math.Angle;
import org.firstinspires.ftc.teamcode.util.math.MathUtil;

public class AngleServoController extends ServoController {
//    public final Angle initialOffset;

    public final Angle minSoftLimit;
    public final Angle maxSoftLimit;

    private Angle targetAngle;

    public AngleServoController(Servo device, String name, Angle totalRotation, double totalGearRatio, Angle minSoftLimit, Angle maxSoftLimit) {
        super(device, name, totalRotation, totalGearRatio);
        this.minSoftLimit = minSoftLimit;
        this.maxSoftLimit = maxSoftLimit;
    }

    public void setTargetAngle(Angle angle) {
        targetAngle = new Angle(MathUtil.clamp(
                angle.getAngle(Angle.AngleNormalization.NONE),
                minSoftLimit.getAngle(Angle.AngleNormalization.NONE),
                maxSoftLimit.getAngle(Angle.AngleNormalization.NONE)
        ));

        device.setPosition(targetAngle.minus(minSoftLimit, Angle.AngleNormalization.NONE).getAngle(Angle.AngleNormalization.NONE) * percentagePerOutputRadian);
    }

    public Angle getTargetAngle() {
        return new Angle(device.getPosition() / percentagePerOutputRadian).plus(minSoftLimit, Angle.AngleNormalization.NONE);
    }

    @Override
    public void update() {

    }

    @Override
    public void addTelemetry(TelemetryManager telemetry) {
        super.addTelemetry(telemetry);
        telemetry.addData("Angle", targetAngle.getAngle(Angle.AngleNormalization.NONE));
    }
}
