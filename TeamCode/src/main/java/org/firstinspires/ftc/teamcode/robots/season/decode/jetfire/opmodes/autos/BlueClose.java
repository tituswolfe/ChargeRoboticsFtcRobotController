package org.firstinspires.ftc.teamcode.robots.season.decode.jetfire.opmodes.autos;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.BezierPoint;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.robots.base.opmodes.BaseAuto;
import org.firstinspires.ftc.teamcode.robots.season.decode.jetfire.JetfireRobot;

@Autonomous(preselectTeleOp = "Jetfire")
public class BlueClose extends BaseAuto<JetfireRobot> {
    Pose startPose = new Pose(-57.24, -43.5, Math.toRadians(-127.5));

    Pose shootPreloadPose = new Pose(-16.57, -22, Math.toRadians(-45));
    Pose line2Control = new Pose(16, -24);
    Pose line2End = new Pose(9.5, -43, Math.toRadians(-90));

    Pose gateControlPoint = new Pose(15, -40);
    Pose openGatePose = new Pose(11.1, -54.73, Math.toRadians(-118.6));

    Pose tunnelIntake = new Pose(11.1 + 5, -54.73, Math.toRadians(-118.6));

    Pose shootBeforeLine1 = new Pose(-16.57, -22, Math.toRadians(-90));

    Pose line1End = new Pose(-10, -43, Math.toRadians(-90));
    Pose line1Shoot = new Pose(-14, -21, Math.toRadians(-90));

    Pose endPose = new Pose(4.2, -42, Math.toRadians(-90));

    PathChain shootPreload;

    PathChain intakeLine2;
    PathChain shootLine2;


    PathChain intakeLine1;
    PathChain shootLine1;
    PathChain gotoEndPose;

    PathChain openGate;
    PathChain intakeTunnel;
    PathChain shootGate;

    int returnPathState = 0;

    public static double endPathTValue = 0.97;

    @Override
    public void autonomousPathUpdate(int pathState) {
        switch (pathState) {
            // START
            case 0:
                robot.getFollower().followPath(shootPreload);
                robot.toggleSubsystems(true);
                setPathState(1, true);
                break;
            // SHOOT PRELOAD
            case 1:
                if (actionTimer.getElapsedTime() > 400 && robot.isFlywheelReady()) {
                    returnPathState = 2;
                    setPathState(20, true);
                }
                break;
            // CYCLE LINE 2
            case 2:
                returnPathState = 4;
                setPathState(40, true);
                break;
            // CYCLE GATE
            case 4:
                returnPathState = 5;
                setPathState(30, true);
                break;
            // CYCLE GATE
            case 5:
                returnPathState = 6;
                setPathState(30, true);
                break;
            // CYCLE LINE 1 (INSIDE THE LINE)
            case 6:
                returnPathState = -1;
                setPathState(50, true);
                break;



            case 40:
                robot.setAutoAimTurntable(false);
                nextPath(intakeLine2, 41);
                break;
            case 41:
                if (robot.getFollower().getCurrentTValue() > 0.9)
                    robot.setAutoAimTurntable(true);

                nextPath(shootLine2, 42);
                break;
            // SHOOT LINE 2
            case 42:
                if (robot.getFollower().getCurrentTValue() > endPathTValue) {
                    setPathState(20, true);
                }
                break;

            case 50:
                robot.setAutoAimTurntable(false);
                nextPath(intakeLine1, 51);
                break;
            case 51:
                if (robot.getFollower().getCurrentTValue() > 0.9)
                    robot.setAutoAimTurntable(true);

                nextPath(shootLine1, 42);
                break;


            case -1:
                if (!robot.getFollower().isBusy()){
                    robot.toggleSubsystems(false);
                    robot.getFollower().followPath(gotoEndPose);
                    setPathState(-2, true);
                }
                break;


            // SHOOT ARTIFACTS
            case 20:
//                if (!robot.isArtifactLoaded()) {
//                    setPathState(returnPathState, true);
//                    break;
//                }

                if (robot.isReadyToShoot() && actionTimer.getElapsedTime() > 100) {
                    robot.fire();
                    setPathState(21, true);
                }
                break;
            case 21:
                if (robot.isReadyToShoot()) {
                    robot.fire();
                    setPathState(22, true);
                }
                break;
            case 22:
                if (robot.isReadyToShoot()) {
                    robot.fire();
                    setPathState(23, true);
                }
                break;
            case 23:
                if (actionTimer.getElapsedTime() > 100) {
                    setPathState(returnPathState, true);
                }
                break;



            // CYCLE GATE
            case 30:
                robot.setAutoAimTurntable(false);
                robot.getFollower().followPath(openGate);
                setPathState(31, true);
                break;
            case 31:
                if (robot.getFollower().getCurrentTValue() > endPathTValue) {
                    setPathState(32, true);
                }
                break;
            case 32:
                if (actionTimer.getElapsedTime() > 1500) {
                    robot.getFollower().followPath(intakeTunnel, true);
                    setPathState(33, true);
                }
                break;
            case 33:
                if (actionTimer.getElapsedTime() > 500) {
                    robot.getFollower().followPath(shootGate, true);
                    setPathState(34, true);
                }
                break;
            case 34:
                if (robot.getFollower().getCurrentTValue() > 0.8) {
                    robot.setAutoAimTurntable(true);
                }

                if (robot.getFollower().getCurrentTValue() > endPathTValue) {
                    setPathState(20, true);
                }
                break;
        }
    }

    public void nextPath(PathChain path, int pState) {
        if (robot.getFollower().getCurrentTValue() > endPathTValue) {
            robot.getFollower().followPath(path);
            setPathState(pState, true);
        }
    }

    @Override
    public void buildPaths(Follower follower) {
        shootPreload = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPreloadPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPreloadPose.getHeading())
                .setBrakingStrength(0.25)
                .build();

        intakeLine2 = follower.pathBuilder()
                .addPath(new BezierCurve(shootPreloadPose, line2Control, line2End))
                .setLinearHeadingInterpolation(shootPreloadPose.getHeading(), line2End.getHeading())
                .build();

        shootLine2 =  follower.pathBuilder()
                .addPath(new BezierCurve(line2End, line2Control, shootPreloadPose))
                .setLinearHeadingInterpolation(line2End.getHeading(), shootPreloadPose.getHeading())
                .setBrakingStrength(0.25)
                .build();

        openGate = follower.pathBuilder()
                .addPath(new BezierCurve(shootPreloadPose, gateControlPoint, openGatePose))
                .setLinearHeadingInterpolation(shootPreloadPose.getHeading(), openGatePose.getHeading())
                //.setBrakingStrength(0.5)
                .build();

        intakeTunnel = follower.pathBuilder()
                .addPath(new BezierLine(openGatePose, tunnelIntake))
                .setLinearHeadingInterpolation(openGatePose.getHeading(), tunnelIntake.getHeading())
                .build();

        shootGate = follower.pathBuilder()
                .addPath(new BezierCurve(tunnelIntake, gateControlPoint, shootBeforeLine1))
                .setLinearHeadingInterpolation(tunnelIntake.getHeading(), shootBeforeLine1.getHeading())
                .setBrakingStrength(0.25)
                .build();

        intakeLine1 = follower.pathBuilder()
                .addPath(new BezierLine(shootBeforeLine1, line1End))
                .setLinearHeadingInterpolation(shootBeforeLine1.getHeading(), line1End.getHeading())
                .build();


        shootLine1 = follower.pathBuilder()
                .addPath(new BezierLine(line1End, line1Shoot))
                .setLinearHeadingInterpolation(line1End.getHeading(), line1Shoot.getHeading())
                .setBrakingStrength(0.25)
                .build();

        gotoEndPose = follower.pathBuilder()
                .addPath(new BezierLine(line1Shoot, endPose))
                .setLinearHeadingInterpolation(line1Shoot.getHeading(), endPose.getHeading())
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
