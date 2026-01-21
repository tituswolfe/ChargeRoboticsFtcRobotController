package org.firstinspires.ftc.teamcode.hardware.vision;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.ftc.localization.RevHubIMU;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

@Configurable
public class LimelightHandler {
    private final Limelight3A limelight;
    private LLResult lastResult;

    public static double X_OFFSET_INCH = 0;
    public static double Y_OFFSET_INCH = 0;

    public LimelightHandler(Limelight3A limelight) {
        this.limelight = limelight;
    }

    public void init(int pipeline) {
        limelight.pipelineSwitch(pipeline);
        limelight.start();
    }

    public void update(double robotYawDeg) {
        if (!limelight.isRunning()) {
            limelight.start();
        }
        limelight.updateRobotOrientation(robotYawDeg);
        lastResult = limelight.getLatestResult();
    }

    public Pose getPose() {
        if (!hasResult()) {
            return null;
        }
        lastResult.getCaptureLatency()


        Pose3D pose3D = lastResult.getBotpose_MT2();

        double xInches = DistanceUnit.INCH.fromMeters(pose3D.getPosition().x) + X_OFFSET_INCH;
        double yInches = DistanceUnit.INCH.fromMeters(pose3D.getPosition().y) + Y_OFFSET_INCH;
        double heading = pose3D.getOrientation().getYaw(AngleUnit.RADIANS);

        return new Pose(xInches, yInches, heading);
    }

    public boolean hasResult() {
        return lastResult != null && lastResult.isValid();
    }

    public Limelight3A getLimelight() {
        return limelight;
    }
}
