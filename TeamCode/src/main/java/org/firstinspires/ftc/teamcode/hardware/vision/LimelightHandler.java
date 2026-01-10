package org.firstinspires.ftc.teamcode.hardware.vision;

import com.pedropathing.ftc.localization.RevHubIMU;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

public class LimelightHandler {
    private final Limelight3A limelight;
    private LLResult lastResult;

    public LimelightHandler(Limelight3A limelight) {
        this.limelight = limelight;
    }

    public void init(int pipeline) {
        limelight.pipelineSwitch(pipeline);
        limelight.start();
    }

    public void update(double robotYawDegrees) {
        limelight.updateRobotOrientation(robotYawDegrees);
        lastResult = limelight.getLatestResult();
    }

    public Pose getPose() {
        if (lastResult == null || !lastResult.isValid()) {
            return null;
        }

        Pose3D pose3D = lastResult.getBotpose(); // TODO: MT2

        return new Pose(pose3D.getPosition().x * -39.37, pose3D.getPosition().y * -39.37, getPose().getHeading());
    }

    public Limelight3A getLimelight() {
        return limelight;
    }
}
