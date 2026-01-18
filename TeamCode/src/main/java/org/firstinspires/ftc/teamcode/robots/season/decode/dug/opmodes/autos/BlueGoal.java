//package org.firstinspires.ftc.teamcode.robots.season.decode.dug.opmodes.autos;
//
//import com.pedropathing.follower.Follower;
//import com.pedropathing.geometry.BezierCurve;
//import com.pedropathing.geometry.BezierLine;
//import com.pedropathing.geometry.Pose;
//import com.pedropathing.paths.PathChain;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//
//import org.firstinspires.ftc.teamcode.robots.base.opmodes.BaseAuto;
//import org.firstinspires.ftc.teamcode.robots.season.decode.dug.DugRobot;
//
//@Autonomous(name = "Blue Goal", group = "dug", preselectTeleOp = "Dug")
//public class BlueGoal extends BaseAuto<DugRobot> {
//    Pose startPose = new Pose(-60, -45, Math.toRadians(-124));
//    Pose shootClosePose = new Pose(-33, -18.8, Math.toRadians(-135));
//    Pose startIntakeLine1Pose = new Pose(-9.85, -20, Math.toRadians(90));
//    Pose finishIntakeLine1Pose = new Pose(-9.85, -48, Math.toRadians(90));
//
//    Pose startIntakeLine2Pose = new Pose(12.2, -20, Math.toRadians(90));
//    Pose finishIntakeLine2Pose = new Pose(15.2, -55, Math.toRadians(90));
//    Pose autoEndPose = new Pose(-5, -35, Math.toRadians(0));
//
//    PathChain shoot1;
//    PathChain intakeLine1;
//    PathChain intakeLine2;
//    PathChain shoot2;
//    PathChain shoot3;
//    PathChain endAuto;
//
//    private int returnPathState = 0;
//
//    @Override
//    public void autonomousPathUpdate(int pathState) {
//        switch (pathState) {
//            // START / GO TO SHOOT POS
//            case 0:
//                robot.setFlywheelSpeedMode(DugRobot.FlywheelSpeedMode.AUTO);
//                robot.setIntakeMode(DugRobot.IntakeMode.INTAKE);
//
//                robot.getFollower().followPath(shoot1, true);
//
//                setPathState(1, true);
//                break;
//            // SHOOT ARTIFACTS #1
//            case 1:
//                if (!robot.getFollower().isBusy()) {
//                    returnPathState = 2;
//                    setPathState(20, true);
//                }
//                break;
//            // INTAKE LINE #1
//            case 2:
//                if (actionTimer.getElapsedTimeSeconds() > 0.25) {
//                    robot.getFollower().followPath(intakeLine1, 0.5, true);
//                    setPathState(3, true);
//                }
//                break;
//            // GO TO SHOOT POS
//            case 3:
//                if (!robot.getFollower().isBusy()) {
//                    robot.getFollower().followPath(shoot2, true);
//                    setPathState(4, true);
//                }
//                break;
//            // SHOOT #2
//            case 4:
//                if (!robot.getFollower().isBusy()) {
//                    returnPathState = 5;
//                    setPathState(20, true);
//                }
//                break;
//            // INTAKE LINE #2
//            case 5:
//                if (actionTimer.getElapsedTimeSeconds() > 0.25) {
//                    robot.getFollower().followPath(intakeLine2, 1, true);
//                    setPathState(6, true);
//                }
//                break;
//            // GO TO SHOOT POS
//            case 6:
//                if (!robot.getFollower().isBusy()) {
//                    robot.getFollower().followPath(shoot3, true);
//                    setPathState(7, true);
//                }
//                break;
//            // SHOOT #3
//            case 7:
//                if (!robot.getFollower().isBusy()) {
//                    returnPathState = -1;
//                    setPathState(20, true);
//                }
//                break;
//
//
//            // - SHOOT THREE ARTIFACTS -
//            // SHOOT #1
//            case 20:
//                if (robot.areFlywheelsReady()) {
//                    robot.launchArtifact();
//                    setPathState(21, true);
//                }
//                break;
//            // PUSH
//            case 21:
//                if (actionTimer.getElapsedTimeSeconds() > 0.25) {
//                    robot.pushArtifact();
//                    setPathState(22, true);
//                }
//                break;
//            // SHOOT #2
//            case 22:
//                if (robot.areFlywheelsReady() && actionTimer.getElapsedTimeSeconds() > 0.25) {
//                    robot.launchArtifact();
//                    setPathState(23, true);
//                }
//                break;
//            // PUSH
//            case 23:
//                if (actionTimer.getElapsedTimeSeconds() > 2) {
//                    robot.pushArtifact();
//                    setPathState(24, true);
//                }
//                break;
//            // SHOOT #3
//            case 24:
//                if (robot.areFlywheelsReady() && actionTimer.getElapsedTimeSeconds() > 0.25) {
//                    robot.launchArtifact();
//                    setPathState(returnPathState, true);
//                }
//                break;
//
//            // END AUTO
//            case -1:
//                if (actionTimer.getElapsedTimeSeconds() > 1 && !robot.getFollower().isBusy()) {
//                    robot.getFollower().followPath(endAuto, true);
//                    setPathState(-2, true);
//                }
//            case -2:
//                if (!robot.getFollower().isBusy()) {
//                    robot.setFlywheelSpeedMode(DugRobot.FlywheelSpeedMode.OFF);
//                    robot.setIntakeMode(DugRobot.IntakeMode.OFF);
//                }
//                break;
//        }
//    }
//
//    @Override
//    public void buildPaths(Follower follower) {
//        // TODO: Runnable runnable = (Runnable) follower.pathBuilder();
//
//        shoot1 = follower.pathBuilder()
//                .addPath(new BezierLine(startPose, shootClosePose))
//                .setLinearHeadingInterpolation(startPose.getHeading(), shootClosePose.getHeading())
//                .build();
//
//        intakeLine1 = follower.pathBuilder()
//                .addPath(new BezierLine(shootClosePose, startIntakeLine1Pose))
//                .setLinearHeadingInterpolation(shootClosePose.getHeading(), startIntakeLine1Pose.getHeading())
//                .addPath(new BezierLine(startIntakeLine1Pose, finishIntakeLine1Pose))
//                .setLinearHeadingInterpolation(startIntakeLine1Pose.getHeading(), finishIntakeLine1Pose.getHeading())
//                .build();
//
//        intakeLine2 = follower.pathBuilder()
//                .addPath(new BezierLine(shootClosePose, startIntakeLine2Pose))
//                .setLinearHeadingInterpolation(shootClosePose.getHeading(), startIntakeLine2Pose.getHeading())
//                .addPath(new BezierLine(startIntakeLine2Pose, finishIntakeLine2Pose))
//                .setLinearHeadingInterpolation(startIntakeLine2Pose.getHeading(), finishIntakeLine2Pose.getHeading())
//                .build();
//
//        shoot2 = follower.pathBuilder()
//                .addPath(new BezierLine(finishIntakeLine1Pose, shootClosePose))
//                .setLinearHeadingInterpolation(finishIntakeLine1Pose.getHeading(), shootClosePose.getHeading())
//                .build();
//
//        shoot3 = follower.pathBuilder()
//                .addPath(new BezierCurve(finishIntakeLine2Pose, new Pose(0, -24), shootClosePose))
//                .setLinearHeadingInterpolation(finishIntakeLine2Pose.getHeading(), shootClosePose.getHeading())
//                .build();
//
//        endAuto = follower.pathBuilder()
//                .addPath(new BezierLine(shootClosePose, autoEndPose))
//                .setLinearHeadingInterpolation(shootClosePose.getHeading(), autoEndPose.getHeading())
//                .build();
//    }
//
//    @Override
//    protected DugRobot instantiateRobot() {
//        return new DugRobot();
//    }
//
//    @Override
//    protected Pose instantiateStartPose() {
//        return startPose;
//    }
//
//    @Override
//    protected AllianceColor instantiateAllianceColor() {
//        return AllianceColor.BLUE;
//    }
//}
