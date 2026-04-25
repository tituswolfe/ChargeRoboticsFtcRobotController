package org.firstinspires.ftc.teamcode.robots.season.decode.jetfire.opmodes.autos.worlds;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.robots.base.opmodes.BaseAuto;
import org.firstinspires.ftc.teamcode.robots.season.decode.jetfire.JetfireRobot;

import java.util.ArrayList;

@Autonomous(name = "CHARGERS (BLUE+FAR)", preselectTeleOp = "Jetfire")
public class BlueFar extends BaseAuto<JetfireRobot> {
    static Pose startPose = new Pose(58, 9.1, Math.toRadians(90));
    static  Pose shootPose = new Pose(55.6, 19.1, Math.toRadians(118.5));

    static Pose spike3Start = new Pose(40.3, 32.8, Math.toRadians(150));
    static Pose spike3End = new Pose(21.3, 36.5, Math.toRadians(-180));

    static Pose humanPlayerStart = new Pose(35.6, 10, Math.toRadians(-180));
    static Pose humanPlayerEnd = humanPlayerStart.withX(20);

    static Pose tunnelStart = humanPlayerStart.withX(humanPlayerStart.getX() + 16);
    static Pose tunnelEnd = tunnelStart.withX(20);

    Pose endPose = new Pose(27, 65, Math.toRadians(90));

    PathChain startToShoot;
    PathChain shootSpike3;
    PathChain shootHumanPlayer;
    PathChain shootTunnel;


    @Override
    public void autonomousPathUpdate(int pathState) {
        switch (pathState) {
            case 0:
                robot.toggleSubsystems(true);
                robot.getFollower().followPath(startToShoot);
                nextPathState();
                break;
            case 1:
                if (robot.isFlywheelReady() && actionTimer.getElapsedTime() > 250) {
                    robot.fire();
                    nextPathState();
                }
                break;
            case 2:
                if (!robot.getRapidFireActionSequence().isRunning()) {
                    robot.getFollower().followPath(shootSpike3);
                    nextPathState();
                }
                break;
            case 3:
                if (robot.isAtEndOfCurrentPath()) {
                    robot.fire();
                    nextPathState();
                }
                break;
            case 4:
                if (!robot.getRapidFireActionSequence().isRunning()) {
                    robot.getFollower().followPath(shootHumanPlayer);
                    nextPathState();
                }
                break;
            case 5:
                if (robot.isAtEndOfCurrentPath()) {
                    robot.fire();
                    nextPathState();
                }
                break;
            case 6:
                if (!robot.getRapidFireActionSequence().isRunning()) {
                    robot.getFollower().followPath(shootTunnel);
                    nextPathState();
                }
                break;
            case 7:
                if (robot.isAtEndOfCurrentPath()) {
                    robot.fire();
                    setPathState(-1);
                }
                break;
//
//            case -1:
//                if (!robot.getRapidFireActionSequence().isRunning()) {
//                    robot.getFollower().followPath(shootToEnd);
//                    setPathState(-2);
//                }
//                break;
        }
    }

    @Override
    public ArrayList<Pose> generatePoses() {
        return null;
    }


    @Override
    public void buildPaths(Follower follower) {
        startToShoot = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();

        shootSpike3 = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose, spike3Start, spike3End))
                .setTangentHeadingInterpolation()
                //.setLinearHeadingInterpolation(shootPose.getHeading(), spike3End.getHeading())
                .addPath(new BezierCurve(spike3End, spike3Start, shootPose))
                .setTangentHeadingInterpolation().setReversed()
                //.setLinearHeadingInterpolation(spike3End.getHeading(), shootPose.getHeading())
                .build();

        shootHumanPlayer = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, humanPlayerStart))
                .setLinearHeadingInterpolation(shootPose.getHeading(), humanPlayerStart.getHeading())
                .addPath(new BezierLine(humanPlayerStart, humanPlayerEnd))
                .setLinearHeadingInterpolation(humanPlayerStart.getHeading(), humanPlayerEnd.getHeading())
                .addPath(new BezierLine(humanPlayerEnd, shootPose))
                .setLinearHeadingInterpolation(humanPlayerStart.getHeading(), shootPose.getHeading())
                .build();

        shootTunnel = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, tunnelStart))
                .setLinearHeadingInterpolation(shootPose.getHeading(), tunnelStart.getHeading())
                .addPath(new BezierLine(tunnelStart, tunnelEnd))
                .setLinearHeadingInterpolation(tunnelStart.getHeading(), tunnelEnd.getHeading())
                .addPath(new BezierLine(tunnelEnd, shootPose))
                .setLinearHeadingInterpolation(tunnelEnd.getHeading(), shootPose.getHeading())
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
