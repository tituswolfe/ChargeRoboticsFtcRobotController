package org.firstinspires.ftc.teamcode.hardware.controllers.servo;

import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.util.math.Angle;
import org.firstinspires.ftc.teamcode.util.math.MathUtil;

import java.time.temporal.ValueRange;

public class AngleServoController extends ServoController {
    public final double minAngle;
    public final double maxAngle;

    private double targetAngle;

    public AngleServoController(Servo device, String name, double totalRotation, double totalGearRatio, double minAngle, double maxAngle) {
        super(device, name, totalRotation, totalGearRatio);
        this.minAngle = minAngle;
        this.maxAngle = maxAngle;
    }

    public void setTargetAngle(double angle) {
        targetAngle = Range.clip(angle, minAngle, maxAngle);
    }

    public double getTargetAngle() {
        return targetAngle;
    }

    @Override
    public void update(long deltaTimeNS) {
        double targetPosition = (targetAngle - minAngle) * percentagePerOutputRadian;
        sendPosition(targetPosition);
    }

    @Override
    public void updateTelemetry(TelemetryManager telemetry) {
        super.updateTelemetry(telemetry);
        telemetry.addData(name + " angle", Math.toDegrees(targetAngle));
    }
}
