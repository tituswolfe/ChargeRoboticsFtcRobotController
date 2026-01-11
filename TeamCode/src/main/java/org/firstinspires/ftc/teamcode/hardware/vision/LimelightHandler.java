package org.firstinspires.ftc.teamcode.hardware.vision;

import com.pedropathing.ftc.localization.RevHubIMU;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

public class LimelightHandler {
    private final Limelight3A limelight;
    private LLResult lastResult;
    private static final double HEADING_OFFSET_DEG = 90;

    public LimelightHandler(Limelight3A limelight) {
        this.limelight = limelight;
    }

    public void init(int pipeline) {
        limelight.pipelineSwitch(pipeline);
        // limelight.start();
    }

    public void update(double robotYawDegrees) {
        limelight.updateRobotOrientation(robotYawDegrees + HEADING_OFFSET_DEG);
        lastResult = limelight.getLatestResult();
    }

    public Pose getPose() {
        if (lastResult == null || !lastResult.isValid()) {
            return null;
        }

        Pose3D pose3D = lastResult.getBotpose_MT2();

        double xInches = DistanceUnit.INCH.fromMeters(pose3D.getPosition().y);
        double yInches = -DistanceUnit.INCH.fromMeters(pose3D.getPosition().x);
        double heading = pose3D.getOrientation().getYaw(AngleUnit.RADIANS) - Math.toRadians(HEADING_OFFSET_DEG);

        return new Pose(
                xInches,
                yInches,
                heading
        );
    }

    public Limelight3A getLimelight() {
        return limelight;
    }
}
