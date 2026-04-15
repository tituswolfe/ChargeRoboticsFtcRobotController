package org.firstinspires.ftc.teamcode.hardware.controllers.servo;

import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.hardware.controllers.HardwareController;
import org.firstinspires.ftc.teamcode.util.math.Angle;
import org.firstinspires.ftc.teamcode.util.math.MathUtil;

public abstract class ServoController extends HardwareController<Servo> {
    protected final double totalRotation;

    protected final double totalGearRatio;
    protected final double percentagePerOutputDegree;
    protected final double percentagePerOutputRadian;

    private double lastPosition = -1;
    private static final double DELTA_FILTERING_THRESHOLD = 0.03;

    public ServoController(Servo device, String name, double totalRotation, double totalGearRatio) {
        super(device, name);

        this.totalRotation = totalRotation;
        this.totalGearRatio = totalGearRatio;
        this.percentagePerOutputDegree = (1.0 / Math.toDegrees(totalRotation)) * totalGearRatio;
        this.percentagePerOutputRadian = (1.0 / totalRotation) * totalGearRatio;

        device.getController().pwmDisable();
    }
    // TODO Hardware controller arrays?

    public void start(double startPosition) {
        device.getController().pwmEnable();
        sendPosition(startPosition);
    }

    protected void sendPosition(double position) {
        boolean isWithinDeltaFilteringThreshold = MathUtil.isWithinRange(position, lastPosition, DELTA_FILTERING_THRESHOLD);

        if (!isWithinDeltaFilteringThreshold) {
            device.setPosition(position);
            lastPosition = position;
        }
    }

    @Override
    public void updateTelemetry(TelemetryManager telemetry) {
        super.updateTelemetry(telemetry);
        telemetry.addData(name + " position", lastPosition);
    }
}
