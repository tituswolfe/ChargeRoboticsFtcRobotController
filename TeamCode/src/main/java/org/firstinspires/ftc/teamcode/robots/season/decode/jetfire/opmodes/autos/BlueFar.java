package org.firstinspires.ftc.teamcode.robots.season.decode.jetfire.opmodes.autos;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.robots.base.opmodes.BaseAuto;
import org.firstinspires.ftc.teamcode.robots.season.decode.jetfire.JetfireRobot;

public class BlueFar extends BaseAuto<JetfireRobot> {
    Pose startPose = new Pose();
    Pose shootPose = new Pose();
    Pose offTheLine = new Pose();

    PathChain shootPath;

    @Override
    public void autonomousPathUpdate(int pathState) {
        switch (pathState) {
            case 0:
                robot.startTurret();
                robot.getFollower().followPath(shootPath, true);
                setPathState(-1, true);
                break;
            case 1:
                if (!robot.getFollower().isBusy()) {
                    robot.getShootThreeArtifacts().start();
                }
                break;
            case -1:
                if (robot.getShootThreeArtifacts().isFinished()) {
                    robot.setAutoAimTurntable(false);
                }
                break;
        }
    }

    @Override
    public void buildPaths(Follower follower) {
        shootPath = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();
    }

    @Override
    protected JetfireRobot instantiateRobot() {
        return new JetfireRobot();
    }

    @Override
    protected Pose instantiateStartPose() {
        return new Pose(0, 0, 0);
    }

    @Override
    protected AllianceColor instantiateAllianceColor() {
        return AllianceColor.BLUE;
    }
}
