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
public class BlueClose extends BaseAuto<JetfireRobot> {
    Pose startPose = new Pose(-57.24, -43.5, Math.toRadians(-127.5));

    Pose shootPose = new Pose(-17.5, -20, Math.toRadians(-146));

    Pose secondLineStartPose = new Pose(12.5, -27.5, Math.toRadians(-90));
    Pose secondLineFinishPose = new Pose(12.5, -50.5, Math.toRadians(-90));

    Pose preOpenGatePose = new Pose(10, -44, Math.toRadians(-120));
    Pose openGatePose = new Pose(10, -57, Math.toRadians(-120));


    Pose firstLineStartPose = new Pose(-12.5, -27.5, Math.toRadians(-90));
    Pose firstLineFinishPose = new Pose(-12.5, -40, Math.toRadians(-90));

    Pose finishPose = new Pose(-12, -32, Math.toRadians(180));

    PathChain shootPreload;
    PathChain cycleLine2;
    //PathChain shootPath2;

    PathChain openGatePath;
    PathChain shootFromGatePath;

    int returnPathState = 0;

    // REVERSE INTAKE BETWEEN LINES

    @Override
    public void autonomousPathUpdate(int pathState) {
        switch (pathState) {
            case 0:
                // START
                robot.getFollower().followPath(shootPreload);
                robot.startAllSubsystems();
                setPathState(1, true);
                break;
            case 1:
                // SHOOT
                if (actionTimer.getElapsedTimeSeconds() > 1) {
                    returnPathState = 2;
                    setPathState(20, true);
                }
            case 2:
                // CYCLE LINE 2
                if (!robot.getFollower().isBusy() && actionTimer.getElapsedTime() > 300) { // goes past fire commands
                    robot.getFollower().followPath(cycleLine2);
                    setPathState(3, true);
                }
                break;
            case 3:
                // SHOOT
                if (!robot.getFollower().isBusy() && robot.getFollower().atPose(shootPose, 7, 7)) {
                    returnPathState = 4;
                    setPathState(20, true);
                }
                break;
            case 4:
                // CYCLE GATE
                if (actionTimer.getElapsedTime() > 300) {
                    returnPathState = 5;
                    setPathState(30, true);
                }
            case 5:
                // CYCLE GATE
                if (!robot.getFollower().isBusy() && actionTimer.getElapsedTime() > 300) {
                    returnPathState = -1;
                    setPathState(30, true);
                }

            case -1:
                if(!robot.getFollower().isBusy() && actionTimer.getElapsedTimeSeconds() > 1){
                    robot.setAutoAimTurntable(false);
                    robot.setFlywheelMode(JetfireRobot.FlywheelMode.OFF);
                    robot.setIntakeMode(JetfireRobot.IntakeMode.OFF);
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
                if (robot.isReadyToShoot() || actionTimer.getElapsedTime() > 700) {
                    robot.fire();
                    setPathState(22, true);
                }
                break;
            case 22:
                if (robot.isReadyToShoot() || actionTimer.getElapsedTime() > 1000) {
                    robot.fire();
                    setPathState(returnPathState, true);
                }
                break;

            // CYCLE GATE
            case 30:
                robot.getFollower().followPath(openGatePath);
                setPathState(31, true);
                break;
            case 31:
                if (!robot.getFollower().isBusy() && actionTimer.getElapsedTime() > 2800) {
                    robot.getFollower().followPath(shootFromGatePath);
                    setPathState(32, true);
                }
                break;
            case 32:
                if (!robot.getFollower().isBusy() && robot.getFollower().atPose(shootPose, 7, 7)) {
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
                .addPath(new BezierCurve(secondLineFinishPose, secondLineStartPose, shootPose))
                .setLinearHeadingInterpolation(secondLineFinishPose.getHeading(), secondLineStartPose.getHeading())
                .build();

        openGatePath = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, preOpenGatePose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), preOpenGatePose.getHeading())
                .addPath(new BezierLine(preOpenGatePose, openGatePose))
                .setLinearHeadingInterpolation(preOpenGatePose.getHeading(), openGatePose.getHeading())
                .build();

        shootFromGatePath = follower.pathBuilder()
                .addPath(new BezierLine(openGatePose, preOpenGatePose))
                .setLinearHeadingInterpolation(openGatePose.getHeading(), preOpenGatePose.getHeading())
                .addPath(new BezierLine(preOpenGatePose, shootPose))
                .setLinearHeadingInterpolation(preOpenGatePose.getHeading(), shootPose.getHeading())
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
