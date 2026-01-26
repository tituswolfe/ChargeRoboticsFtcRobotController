package org.firstinspires.ftc.teamcode.hardware.controllers.servo;

import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.hardware.controllers.HardwareController;
import org.firstinspires.ftc.teamcode.util.math.Angle;

public abstract class ServoController extends HardwareController<Servo> {
    protected final Angle totalRotation;

    protected final double totalGearRatio;
    protected final double percentagePerOutputDegree;
    protected final double percentagePerOutputRadian;

    public ServoController(Servo device, String name, Angle totalRotation, double totalGearRatio) {
        super(device, name);

        this.totalRotation = totalRotation;
        this.totalGearRatio = totalGearRatio;
        // TODO: Simplify
        this.percentagePerOutputDegree = (1.0 / totalRotation.getAngle(Angle.AngleUnit.DEGREES, Angle.AngleNormalization.NONE)) * totalGearRatio;
        this.percentagePerOutputRadian = (1.0 / totalRotation.getAngle(Angle.AngleUnit.RADIANS, Angle.AngleNormalization.NONE)) * totalGearRatio;
    }

    public Angle getTotalRotation() {
        return totalRotation;
    }

    public double getTotalGearRatio() {
        return totalGearRatio;
    }

    public double getPercentagePerOutputDegree() {
        return percentagePerOutputDegree;
    }

    public double getPercentagePerOutputRadian() {
        return percentagePerOutputRadian;
    }

    @Override
    public void addTelemetry(TelemetryManager telemetry) {
        super.addTelemetry(telemetry);
        telemetry.addData("Position", device.getPosition());
    }
}
