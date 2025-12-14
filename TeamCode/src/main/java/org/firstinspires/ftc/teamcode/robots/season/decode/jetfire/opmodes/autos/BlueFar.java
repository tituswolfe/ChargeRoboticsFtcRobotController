package org.firstinspires.ftc.teamcode.robots.season.decode.jetfire.opmodes.autos;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.robots.base.opmodes.BaseAuto;
import org.firstinspires.ftc.teamcode.robots.season.decode.jetfire.DugRobot;

@Autonomous(name = "Blue Far", group = "dug", preselectTeleOp = "Dug")
public class BlueFar extends BaseAuto<DugRobot> {
    public Pose startPose = new Pose(57.8, -24, Math.toRadians(-180));
    public Pose shootPose = new Pose(-0.5, -9.2, Math.toRadians(-154.2));
    public Pose endPose = new Pose(33.8, -24, Math.toRadians(-180));


    public PathChain shoot1;

    PathChain endAuto;

    private int returnPathState = 0;

    @Override
    public void autonomousPathUpdate(int pathState) {
        switch (pathState) {
            case 0:
                robot.setIntakeMode(DugRobot.IntakeMode.INTAKE);
                robot.setFlywheelSpeedMode(DugRobot.FlywheelSpeedMode.AUTO);

                robot.getFollower().followPath(shoot1, true);
                setPathState(1, true);
                break;
            case 1:
                if (!robot.getFollower().isBusy()) {
                    returnPathState = -1;
                    setPathState(21, true);
                }
                break;

            case -1:
                if (actionTimer.getElapsedTimeSeconds() > 1 && !robot.getFollower().isBusy()) {
                    robot.getFollower().followPath(endAuto, true);
                    setPathState(-2, true);
                }
            case -2:
                if (!robot.getFollower().isBusy()) {
                    robot.setFlywheelSpeedMode(DugRobot.FlywheelSpeedMode.OFF);
                    robot.setIntakeMode(DugRobot.IntakeMode.OFF);
                }



            // - SHOOT THREE ARTIFACTS -
            // SHOOT #1
            case 20:
                if (robot.areFlywheelsReady()) {
                    robot.launchArtifact();
                    setPathState(21, true);
                }
                break;
            // PUSH
            case 21:
                if (actionTimer.getElapsedTimeSeconds() > 0.25) {
                    robot.pushArtifact();
                    setPathState(22, true);
                }
                break;
            // SHOOT #2
            case 22:
                if (robot.areFlywheelsReady() && actionTimer.getElapsedTimeSeconds() > 0.25) {
                    robot.launchArtifact();
                    setPathState(23, true);
                }
                break;
            // PUSH
            case 23:
                if (actionTimer.getElapsedTimeSeconds() > 2) {
                    robot.pushArtifact();
                    setPathState(24, true);
                }
                break;
            // SHOOT #3
            case 24:
                if (robot.areFlywheelsReady() && actionTimer.getElapsedTimeSeconds() > 0.25) {
                    robot.launchArtifact();
                    setPathState(returnPathState, true);
                }
                break;
        }
    }

    @Override
    public void buildPaths(Follower follower) {
        shoot1 = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();

        endAuto = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, endPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), endPose.getHeading())
                .build();
    }

    @Override
    protected DugRobot instantiateRobot() {
        return new DugRobot();
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
