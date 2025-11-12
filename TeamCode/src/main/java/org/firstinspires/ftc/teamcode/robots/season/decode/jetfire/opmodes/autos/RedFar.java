package org.firstinspires.ftc.teamcode.robots.season.decode.jetfire.opmodes.autos;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.robots.base.opmodes.BaseAuto;
import org.firstinspires.ftc.teamcode.robots.season.decode.jetfire.JetFireRobot;

@Autonomous(name = "Red Far", preselectTeleOp = "JetFire")
public class RedFar extends BaseAuto<JetFireRobot> {
    public Pose startPose = new Pose(57.8, 24, Math.toRadians(-180));
    public Pose shootPose = new Pose(33.8, 24, Math.toRadians(-180));

    public Pose startLine3Pose = new Pose(35, -23, Math.toRadians(90));
    public Pose finishLine3Pose = new Pose(35, -54, Math.toRadians(90));

    public PathChain shootFar;
    public PathChain intakeLine3;

    @Override
    public void autonomousPathUpdate(int pathState) {
        switch (pathState) {
            case 0:
//                robot.startFlywheels(1.3);
//                robot.startIntake();

                robot.getFollower().followPath(shootFar, true);

                setPathState(1, true);
                break;
            case 1:
                if (!robot.getFollower().isBusy()) {
                    setPathState(-1, true);
                }
                break;
//            case 2:
//                if (actionTimer.getElapsedTimeSeconds() > 2) {
//                    robot.launchArtifact();
//                    setPathState(3, true);
//                }
//                break;
//            case 3:
//                if (actionTimer.getElapsedTimeSeconds() > 2) {
//                    robot.launchArtifact();
//                    setPathState(4, true);
//                }
//                break;
//            case 4:
//                if (actionTimer.getElapsedTimeSeconds() > 2) {
//                    robot.launchArtifact();
//                    setPathState(5, true);
//                }
//                break;
//            case 5:
//                if (actionTimer.getElapsedTimeSeconds() > 2) {
//                    robot.getFollower().followPath(intakeLine3);
//                    setPathState(-1, true);
//                }
//                break;
            case -1:
                if (!robot.getFollower().isBusy()) {
                    robot.setFlywheelSpeedMode(JetFireRobot.FlywheelSpeedMode.OFF);
                    robot.setIntakeMode(JetFireRobot.IntakeMode.OFF);
                }
                break;
        }
    }

    @Override
    public void buildPaths(Follower follower) {
        shootFar = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();
//        intakeLine3 = follower.pathBuilder()
//                .addPath(new BezierLine(shootPose, startLine3Pose))
//                .setLinearHeadingInterpolation(shootPose.getHeading(), startLine3Pose.getHeading())
//                .addPath(new BezierLine(startLine3Pose, finishLine3Pose))
//                .setLinearHeadingInterpolation(finishLine3Pose.getHeading(), finishLine3Pose.getHeading())
//                .build();
    }

    @Override
    protected JetFireRobot instantiateRobot() {
        return new JetFireRobot();
    }

    @Override
    protected Pose instantiateStartPose() {
        return startPose;
    }

    @Override
    protected AllianceColor instantiateAllianceColor() {
        return AllianceColor.RED;
    }
}
