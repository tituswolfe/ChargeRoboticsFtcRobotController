package org.firstinspires.ftc.teamcode.robots.season.decode.jetfire.opmodes.autos;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.robots.base.opmodes.BaseAuto;
import org.firstinspires.ftc.teamcode.robots.season.decode.jetfire.JetfireRobot;

public class RedClose extends BaseAuto<JetfireRobot> {
    // Start pose
    // shoot preload pose

    // open gate


    Pose startPose = new Pose(-57.24, 43.5, Math.toRadians(127.5));

    Pose shootPreloadPose = new Pose();
    Pose shootPose = new Pose(-13, 23, Math.toRadians(90));

    Pose secondLineStartPose = new Pose(11.5, 26.5, Math.toRadians(90));

    Pose secondLineStartControl = new Pose();

    Pose secondLineFinishPose = new Pose(11.5, 46, Math.toRadians(90));

    Pose gateToShootControlPoint = new Pose(12, 34);

    Pose preOpenGatePose = new Pose(10.9, 50, Math.toRadians(113));
    Pose openGatePose = new Pose(10.9, 54.73, Math.toRadians(118.6));

    Pose firstLineStartPose = new Pose(-10, 27.5, Math.toRadians(90));
    Pose firstLineFinishPose = new Pose(-10, 45, Math.toRadians(90));

    Pose finishPose = new Pose(-41, 22, Math.toRadians(122.5));

    PathChain shootPreload;
    PathChain cycleLine2;
    PathChain cycleLine1;

    PathChain openGatePath;
    PathChain shootFromGatePath;

    int returnPathState = 0;

    // TODO: Flywheel speed check
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
                if (actionTimer.getElapsedTime() > 1700) {
                    returnPathState = 2;
                    setPathState(20, true);
                }
                break;
            // CYCLE LINE 2
            case 2:
                if (robot.getFollower().getCurrentTValue() > 0.98) {
                    robot.getFollower().followPath(cycleLine2);
                    setPathState(3, true);
                }
                break;
            // SHOOT LINE 2
            case 3:
                if (!robot.getFollower().isBusy()) {
                    returnPathState = 4;
                    setPathState(20, true);
                }
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
                robot.getFollower().followPath(cycleLine1);
                setPathState(7, true);
                break;
            case 7:
                // SHOOT
                if (robot.getFollower().atPose(finishPose, 15, 15)) {
                    returnPathState = -1;
                    setPathState(20, true);
                }
                break;


            case -1:
                if (!robot.getFollower().isBusy()){
                    robot.toggleSubsystems(false);
                    setPathState(-2, true);
                }
                break;


            // SHOOT ARTIFACTS
            case 20:
                if (robot.isReadyToShoot() || actionTimer.getElapsedTime() > 600) {
                    robot.fire();
                    setPathState(21, true);
                }
                break;
            case 21:
                if (robot.isReadyToShoot() || actionTimer.getElapsedTime() > 600) {
                    robot.fire();
                    setPathState(22, true);
                }
                break;
            case 22:
                if (robot.isReadyToShoot() || actionTimer.getElapsedTime() > 600) {
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
                robot.getFollower().followPath(openGatePath);
                setPathState(31, true);
                break;
            case 31:
                if (actionTimer.getElapsedTime() > 3500) {
                    robot.getFollower().followPath(shootFromGatePath);
                    setPathState(32, true);
                }
                break;
            case 32:
                if (!robot.getFollower().isBusy()) {
                    setPathState(20, true);
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

        cycleLine2 = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, secondLineStartPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), secondLineStartPose.getHeading())
                .addPath(new BezierLine(secondLineStartPose, secondLineFinishPose))
                .setConstantHeadingInterpolation(secondLineStartPose.getHeading())
                .addPath(new BezierLine(secondLineFinishPose, secondLineStartPose))
                .setLinearHeadingInterpolation(secondLineFinishPose.getHeading(), secondLineStartPose.getHeading())
                .addPath(new BezierLine(secondLineStartPose, shootPose))
                .setLinearHeadingInterpolation(secondLineStartPose.getHeading(), shootPose.getHeading())
                .build();

//        cycleLine2 = follower.pathBuilder()
//                .addPath(new BezierCurve(shootPreloadPose, secondLineFinishPose, secondLineFinishPose))
//                .setTangentHeadingInterpolation()
//                .addPath(new BezierCurve(secondLineFinishPose, secondLineFinishPose, shootPreloadPose))
//                .setTangentHeadingInterpolation().setReversed()
//                .build();
//
//                .addPath(new BezierLine(shootPose, secondLineStartPose))
//                .setLinearHeadingInterpolation(shootPose.getHeading(), secondLineStartPose.getHeading())
//                .addPath(new BezierLine(secondLineStartPose, secondLineFinishPose))
//                .setConstantHeadingInterpolation(secondLineStartPose.getHeading())
//                .addPath(new BezierLine(secondLineFinishPose, secondLineStartPose))
//                .setLinearHeadingInterpolation(secondLineFinishPose.getHeading(), secondLineStartPose.getHeading())
//                .addPath(new BezierLine(secondLineStartPose, shootPose))
//                .setLinearHeadingInterpolation(secondLineStartPose.getHeading(), shootPose.getHeading())
//                .build();

        cycleLine1 = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, firstLineStartPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), firstLineStartPose.getHeading())
                .addPath(new BezierLine(firstLineStartPose, firstLineFinishPose))
                .setLinearHeadingInterpolation(firstLineStartPose.getHeading(), firstLineFinishPose.getHeading())
                .addPath(new BezierLine(firstLineFinishPose, finishPose))
                .setLinearHeadingInterpolation(firstLineFinishPose.getHeading(), finishPose.getHeading())
                .build();

        openGatePath = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose, gateToShootControlPoint, preOpenGatePose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), preOpenGatePose.getHeading())

                .addPath(new BezierLine(preOpenGatePose, openGatePose))
                .setLinearHeadingInterpolation(preOpenGatePose.getHeading(), openGatePose.getHeading())
                .build();

        shootFromGatePath = follower.pathBuilder()
                .addPath(new BezierCurve(openGatePose, gateToShootControlPoint, shootPose))
                .setLinearHeadingInterpolation(openGatePose.getHeading(), shootPose.getHeading())
                .build();

//        finishPath = follower.pathBuilder()
//                .addPath(new BezierLine(shootPose, finishPose))
//                .setLinearHeadingInterpolation(shootPose.getHeading(), finishPose.getHeading())
//                .build();

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
        return AllianceColor.RED;
    }
}
