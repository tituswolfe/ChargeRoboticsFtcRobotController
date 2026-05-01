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
    Pose startPose = new Pose(58, 9.1, Math.toRadians(90));
    Pose shootPose = new Pose(55.6, 19.1, Math.toRadians(118.5));

    Pose spike3Start = new Pose(40.3, 32.8, Math.toRadians(150));
    Pose spike3End = new Pose(21.3, 36.5, Math.toRadians(-180));

    Pose shootPose2 = new Pose(50, 10, Math.toRadians(-180)); // = new Pose(54.2, 17.3, Math.toRadians(-180));

    //Pose humanPlayerStart = new Pose(22, 10, Math.toRadians(-180));
    Pose humanPlayerEnd = new Pose(20, 10, Math.toRadians(-180)); //humanPlayerStart.withX(20);

    //Pose tunnelStart = new Pose(35.6, 10 + 16, Math.toRadians(-180));
    Pose tunnelEnd = new Pose(20, 10 + 16, Math.toRadians(-180)); //tunnelStart.withX(20);

    Pose endPose = new Pose(54.6, 27.5, Math.toRadians(120));

    PathChain startToShoot;
    PathChain shootSpike3;
    PathChain shootHumanPlayer;
    PathChain shootTunnel;
    PathChain shootToEnd;

    int humanPlayerCycles = 7;

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
                if (robot.getRapidFireActionSequence().isRunning()) {
                    break;
                }

                if (humanPlayerCycles <= 0) {
                    setPathState(-1);
                } else if (humanPlayerCycles % 2 == 0) {
                    robot.getFollower().followPath(shootTunnel);
                    nextPathState();
                } else {
                    robot.getFollower().followPath(shootHumanPlayer);
                    nextPathState();
                }

                break;
            case 5:
                if (robot.isAtEndOfCurrentPath()) {
                    robot.fire();
                    humanPlayerCycles--;
                    setPathState(4);
                }
                break;

            case -1:
                if (!robot.getRapidFireActionSequence().isRunning()) {
                    robot.toggleSubsystems(false);
                    robot.getFollower().followPath(shootToEnd);
                    setPathState(-2);
                }
                break;
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
                .addPath(new BezierCurve(spike3End, spike3Start, shootPose2))
                .setLinearHeadingInterpolation(spike3End.getHeading(), shootPose2.getHeading())
                .build();

        shootHumanPlayer = follower.pathBuilder()
                .addPath(new BezierLine(shootPose2, humanPlayerEnd))
                .setLinearHeadingInterpolation(shootPose2.getHeading(), humanPlayerEnd.getHeading())
                .addPath(new BezierLine(humanPlayerEnd, shootPose2))
                .setLinearHeadingInterpolation(humanPlayerEnd.getHeading(), shootPose2.getHeading())
                .build();

        shootTunnel = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose2, spike3End, tunnelEnd))
                .setLinearHeadingInterpolation(shootPose2.getHeading(), tunnelEnd.getHeading())
                .addPath(new BezierLine(tunnelEnd, shootPose2))
                .setLinearHeadingInterpolation(tunnelEnd.getHeading(), shootPose2.getHeading())
                .build();

        shootToEnd = follower.pathBuilder()
                .addPath(new BezierLine(shootPose2, endPose))
                .setLinearHeadingInterpolation(shootPose2.getHeading(), endPose.getHeading())
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
