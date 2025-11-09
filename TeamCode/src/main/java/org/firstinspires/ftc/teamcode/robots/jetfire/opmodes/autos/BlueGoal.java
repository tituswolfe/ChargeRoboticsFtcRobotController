package org.firstinspires.ftc.teamcode.robots.jetfire.opmodes.autos;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.robots.base.opmodes.BaseAuto;
import org.firstinspires.ftc.teamcode.robots.jetfire.JetFireRobot;

@Autonomous(name = "Blue Goal", group = "decode", preselectTeleOp = "JetFire")
public class BlueGoal extends BaseAuto<JetFireRobot> {
    Pose startPose = new Pose(-60, -45, Math.toRadians(-124));
    Pose shootClosePose = new Pose(-33, -18.8, Math.toRadians(-135));
    Pose startIntakeLine1Pose = new Pose(-9.85, -20, Math.toRadians(90));
    Pose finishIntakeLine1Pose = new Pose(-9.85, -50, Math.toRadians(90));
    Pose autoEndPose = new Pose(-5, -42.7, Math.toRadians(90));

    PathChain shoot1;
    PathChain intakeLine1;
    PathChain shoot2;
    PathChain endAuto;

    @Override
    public void autonomousPathUpdate(int pathState) {
        switch (pathState) {
            case 0:
                robot.startFlywheels(1);
                robot.startIntake();

                robot.getFollower().followPath(shoot1, true);

                setPathState(1, true);
                break;
            case 1:
                if (!robot.getFollower().isBusy()) {
                    setPathState(2, true);
                }
                break;
            case 2:
                if (actionTimer.getElapsedTimeSeconds() > 1) {
                    robot.launchArtifact();
                    setPathState(3, true);
                }
                break;
            case 3:
                if (actionTimer.getElapsedTimeSeconds() > 2) {
                    robot.launchArtifact();
                    setPathState(4, true);
                }
                break;
            case 4:
                if (actionTimer.getElapsedTimeSeconds() > 2) {
                    robot.launchArtifact();
                    setPathState(5, true);
                }
                break;
            case 5:
                if (actionTimer.getElapsedTimeSeconds() > 2) {
                    robot.getFollower().followPath(intakeLine1, true);
                    setPathState(6, true);
                }
                break;
            case 6:
                if (!robot.getFollower().isBusy() && actionTimer.getElapsedTimeSeconds() > 1) {
                    robot.getFollower().followPath(shoot2, true);
                    setPathState(7, true);
                }
                break;
            case 7:
                if (!robot.getFollower().isBusy()) {
                    setPathState(8, true);
                }
                break;
            case 8:
                if (actionTimer.getElapsedTimeSeconds() > 2) {
                    robot.launchArtifact();
                    setPathState(9, true);
                }
                break;
            case 9:
                if (actionTimer.getElapsedTimeSeconds() > 2) {
                    robot.launchArtifact();
                    setPathState(10, true);
                }
                break;
            case 10:
                if (actionTimer.getElapsedTimeSeconds() > 2) {
                    robot.flicker();
                    robot.launchArtifact();
                    setPathState(11, true);
                }
                break;
            case 11:
                if (actionTimer.getElapsedTimeSeconds() > 2) {
                    robot.getFollower().followPath(endAuto, true);
                    setPathState(-1, true);
                }
            case -1:
                if (!robot.getFollower().isBusy()) {
                    robot.stopIntake();
                    robot.stopFlywheels();
                }
                break;
        }
    }

    @Override
    public void buildPaths(Follower follower) {
        // TODO: Runnable runnable = (Runnable) follower.pathBuilder();

        shoot1 = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootClosePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootClosePose.getHeading())
                .build();

        intakeLine1 = follower.pathBuilder()
                .addPath(new BezierLine(shootClosePose, startIntakeLine1Pose))
                .setLinearHeadingInterpolation(shootClosePose.getHeading(), startIntakeLine1Pose.getHeading())
                .addPath(new BezierLine(startIntakeLine1Pose, finishIntakeLine1Pose))
                .setLinearHeadingInterpolation(startIntakeLine1Pose.getHeading(), finishIntakeLine1Pose.getHeading())
                .build();

        shoot2 = follower.pathBuilder()
                .addPath(new BezierLine(finishIntakeLine1Pose, shootClosePose))
                .setLinearHeadingInterpolation(finishIntakeLine1Pose.getHeading(), shootClosePose.getHeading())
                .build();

        endAuto = follower.pathBuilder()
                .addPath(new BezierLine(shootClosePose, autoEndPose))
                .setLinearHeadingInterpolation(shootClosePose.getHeading(), autoEndPose.getHeading())
                .build();
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
        return AllianceColor.BLUE;
    }
}
