package org.firstinspires.ftc.teamcode.hardware.vision;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.hardware.controllers.HardwareController;

@Configurable
public class LimelightController extends HardwareController<Limelight3A> {
    private LLResult latestResult;
    private boolean hasResult = false;
    private Pose LLPose;
    private double latencyMs;

    public static double X_OFFSET_INCH = 0;
    public static double Y_OFFSET_INCH = 0;

    public LimelightController(Limelight3A device, String name) {
        super(device, name);
    }

    public void init(int pipeline) {
        device.pipelineSwitch(pipeline);
        device.start();
    }

    public void updateRobotHeading(double headingDeg) {
        device.updateRobotOrientation(headingDeg);
    }

    @Override
    public void update() {
        if (!device.isRunning()) {
            device.start();
        }

        latestResult = device.getLatestResult();
        //device.captureSnapshot("a");

        if (latestResult == null || !latestResult.isValid()) {
            hasResult = false;
            latencyMs = -1;
            LLPose = new Pose();
            return;
        };

        latencyMs = latestResult.getCaptureLatency() + latestResult.getParseLatency() + latestResult.getTargetingLatency();

        Pose3D pose3D = latestResult.getBotpose_MT2();

        double xInches = DistanceUnit.INCH.fromMeters(pose3D.getPosition().x) + X_OFFSET_INCH;
        double yInches = DistanceUnit.INCH.fromMeters(pose3D.getPosition().y) + Y_OFFSET_INCH;
        double heading = pose3D.getOrientation().getYaw(AngleUnit.RADIANS);

        LLPose = new Pose(xInches, yInches, heading);
    }

    public boolean hasResult() {
        return hasResult;
    }

    public Pose getPose() {
        return LLPose;
    }

    public double getLatencyMs() {
        return latencyMs;
    }

    @Override
    public void addTelemetry(TelemetryManager telemetry) {
        super.addTelemetry(telemetry);
        telemetry.addData("Connected", device.isConnected());
        telemetry.addData("Has Result", hasResult);
        telemetry.addData("Botpose MT2", LLPose);
        telemetry.addData("Latency (MS)", latencyMs);
    }
}
