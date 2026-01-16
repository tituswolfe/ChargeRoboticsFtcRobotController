package org.firstinspires.ftc.teamcode.robots.season.decode.jetfire.opmodes.autos;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.robots.base.opmodes.BaseAuto;
import org.firstinspires.ftc.teamcode.robots.season.decode.jetfire.JetfireRobot;

@Autonomous(preselectTeleOp = "Jetfire")
public class BlueFar extends BaseAuto<JetfireRobot> {
    Pose startPose = new Pose(63, -14.1, Math.toRadians(-180));

    Pose shootPose = new Pose(53, -17, Math.toRadians(-159.1));

    Pose intakeLine3ControlPoint = new Pose(37, -21);
    Pose intakeLine3Finish = new Pose(33, -53.2, Math.toRadians(-90));

    Pose intakeHumanPlayerLine = new Pose(59.3, -59.1, Math.toRadians(-90));

    Pose intakeFromHumanPlayerStart = new Pose(58, -57, Math.toRadians(-120));
    Pose intakeFromHumanPlayerFinish = new Pose(28, -57, Math.toRadians(-120));
    Pose intakeFromHumanPlayerControlPoint = new Pose(-20, 56.8, Math.toRadians(-135));

    Pose endPose = new Pose(60, -35, Math.toRadians(-180));

    PathChain shootPreload;
    PathChain cycleLine3;
    PathChain cycleHumanPlayer;

    PathChain waitForGate;
    PathChain shootFromGate;



    int returnPathState = 0;

    @Override
    public void autonomousPathUpdate(int pathState) {
        if (opmodeTimer.getElapsedTimeSeconds() >= 27.5 && pathState > 0) {
            setPathState(-1, true);
            return;
        }

        switch (pathState) {
            case 0:
                // START
                robot.getFollower().followPath(shootPreload);
                robot.startAllSubsystems();
                setPathState(1, true);
                break;
            case 1:
                // SHOOT
                if (actionTimer.getElapsedTime() > 2000) {
                    returnPathState = 2;
                    setPathState(20, true);
                }
                break;
            case 2:
                // CYCLE LINE 2
                if (!robot.getFollower().isBusy()) {
                    robot.getFollower().followPath(cycleLine3);
                    setPathState(3, true);
                }
                break;
            case 3:
                // SHOOT
                if (!robot.getFollower().isBusy()) {
                    returnPathState = 4;
                    setPathState(20, true);
                }
                break;
            case 4:
                robot.getFollower().followPath(cycleHumanPlayer);
                setPathState(5, true);
            case 5:
                // SHOOT
                if (!robot.getFollower().isBusy()) {
                    returnPathState = 6;
                    setPathState(20, true);
                }
                break;
            case 6:
                returnPathState = 7;
                setPathState(30, true);
                break;
            case 7:
                returnPathState = -1;
                setPathState(30, true);
                break;

            case -1:
                if(!robot.getFollower().isBusy() ){
                    robot.setAutoAimTurntable(false);
                    robot.setFlywheelMode(JetfireRobot.FlywheelMode.OFF);
                    robot.setIntakeMode(JetfireRobot.IntakeMode.OFF);

                    robot.getFollower().holdPoint(endPose);
                    setPathState(-2, true);
                }
                break;


            // CYCLE GATE
            case 30:
                robot.getFollower().followPath(waitForGate);
                setPathState(31, true);
                break;
            case 31:
                if (!robot.getFollower().isBusy() && (robot.isArtifactLoaded() || actionTimer.getElapsedTime() > 2000)) {
                    setPathState(32, true);
                }
                break;
            case 32:
                if (actionTimer.getElapsedTime() > 1750) {
                    robot.getFollower().followPath(shootFromGate);
                    setPathState(33, true);
                }
                break;
            case 33:
                // SHOOT
                if (!robot.getFollower().isBusy()) {
                    setPathState(20, true);
                }
                break;



            // SHOOT ARTIFACTS
            case 20:
                if (robot.isReadyToShoot() || actionTimer.getElapsedTime() > 500) {
                    robot.fire();
                    setPathState(21, true);
                }
                break;
            case 21:
                if (robot.isReadyToShoot() || actionTimer.getElapsedTime() > 500) {
                    robot.fire();
                    setPathState(22, true);
                }
                break;
            case 22:
                if (robot.isReadyToShoot() || actionTimer.getElapsedTime() > 500) {
                    robot.fire();
                    setPathState(23, true);
                }
                break;
            case 23:
                if (actionTimer.getElapsedTime() > 100) {
                    setPathState(returnPathState, true);
                }
                break;
        }
    }

    @Override
    public void buildPaths(Follower follower) {
        shootPreload = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();

        cycleLine3 = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose, intakeLine3ControlPoint, intakeLine3Finish))
                .setTangentHeadingInterpolation()
                .addPath(new BezierCurve(intakeLine3Finish, intakeLine3ControlPoint, shootPose))
                .setTangentHeadingInterpolation().setReversed()
                .build();

        cycleHumanPlayer = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, intakeHumanPlayerLine))
                .setLinearHeadingInterpolation(shootPose.getHeading(), intakeHumanPlayerLine.getHeading())
                .addPath(new BezierLine(intakeHumanPlayerLine, shootPose))
                .setLinearHeadingInterpolation(intakeHumanPlayerLine.getHeading(), shootPose.getHeading())
                .build();

        waitForGate = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, intakeFromHumanPlayerStart))
                .setLinearHeadingInterpolation(shootPose.getHeading(), intakeFromHumanPlayerStart.getHeading())
                .addPath(new BezierLine(intakeFromHumanPlayerStart, intakeFromHumanPlayerFinish))
                .setLinearHeadingInterpolation(intakeFromHumanPlayerStart.getHeading(), intakeFromHumanPlayerFinish.getHeading())
                .build();

        shootFromGate = follower.pathBuilder()
                .addPath(new BezierLine(intakeFromHumanPlayerFinish, shootPose))
                .setLinearHeadingInterpolation(intakeFromHumanPlayerFinish.getHeading(), shootPose.getHeading())
                .build();

    }

    @Override
    protected JetfireRobot instantiateRobot() {
        return new JetfireRobot();
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
