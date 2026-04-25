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

@Autonomous(name = "X BOTS (RED+CLOSE)", preselectTeleOp = "Jetfire")
public class RedClose extends BaseAuto<JetfireRobot> {
    Pose startPose = new Pose(20, 120, Math.toRadians(143.8)).mirror();
    Pose shootWithBack = new Pose(53.6, 80.8, Math.toRadians(-142.4)).mirror();

    Pose spike2Start = new Pose(41.3, 59.7, Math.toRadians(-180)).mirror();
    Pose spike2End = new Pose(41.3 - 15, 59.7, Math.toRadians(-180)).mirror();

    Pose gateControl = new Pose(36, 70, Math.toRadians(-150)).mirror();
    Pose gateOpen = new Pose (13.8, 58.5, Math.toRadians(147.7)).mirror();

    Pose intakeTunnelControl = new Pose(18, 59, Math.toRadians(180)).mirror();
    Pose intakeTunnel = new Pose(124, 59, Math.toRadians(0));

    Pose spike1Start = new Pose(42.2, 83.3, Math.toRadians(-180)).mirror();
    Pose spike1End = new Pose(42.2 - 20, 83.3, Math.toRadians(-180)).mirror();

    Pose endPose = new Pose(27, 65, Math.toRadians(90)).mirror();

    PathChain startToShoot;

    PathChain shootSpike2;

    PathChain shootToGate;
    PathChain gateToShoot;

    PathChain shootSpike1;

    PathChain shootToEnd;

    int gateCycleAttempts = 3;

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
                if (robot.isAtEndOfCurrentPath()) {
                    robot.getFollower().followPath(shootSpike2);
                    nextPathState();
                }
                break;
            case 3:
                if (robot.isAtEndOfCurrentPath()) {
                    robot.fire();
                    setPathState(4);
                }
                break;
            case 4:
                if (gateCycleAttempts <= 0) {
                    setPathState(8);
                } else if (!robot.getRapidFireActionSequence().isRunning()) {
                    robot.getFollower().followPath(shootToGate);
                    setPathState(5);
                }
                break;
            case 5:
                if (robot.isAtEndOfCurrentPath()) {
                    setPathState(6);
                }
                break;
            case 6:
                if (actionTimer.getElapsedTime() > 1400) {
                    robot.getFollower().followPath(gateToShoot);
                    setPathState(7);
                }
                break;
            case 7:
                if (robot.isAtEndOfCurrentPath()) {
                    robot.fire();
                    gateCycleAttempts--;
                    setPathState(4);
                }
                break;
            case 8:
                if (robot.isAtEndOfCurrentPath()) {
                    robot.getFollower().followPath(shootSpike1);
                    nextPathState();
                }
                break;
            case 9:
                if (robot.isAtEndOfCurrentPath()) {
                    robot.fire();
                    setPathState(-1);
                }
                break;
            case -1:
                if (!robot.getRapidFireActionSequence().isRunning()) {
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
                .addPath(new BezierLine(startPose, shootWithBack))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootWithBack.getHeading())
                .build();

        shootSpike2 = follower.pathBuilder()
                .addPath(new BezierCurve(shootWithBack, spike2Start, spike2End))
                .setLinearHeadingInterpolation(shootWithBack.getHeading(), spike2Start.getHeading())
                .addPath(new BezierCurve(spike2End, spike2Start, shootWithBack))
                .setLinearHeadingInterpolation(spike2End.getHeading(), shootWithBack.getHeading())
                .build();

        shootToGate = follower.pathBuilder()
                .addPath(new BezierCurve(shootWithBack, gateControl, gateOpen))
                .setLinearHeadingInterpolation(shootWithBack.getHeading(), gateOpen.getHeading())
                .build();

        gateToShoot = follower.pathBuilder()
                .addPath(new BezierLine(gateOpen, intakeTunnelControl))
                .setLinearHeadingInterpolation(gateOpen.getHeading(), intakeTunnelControl.getHeading())
                .addPath(new BezierLine(intakeTunnelControl, intakeTunnel))
                .setLinearHeadingInterpolation(intakeTunnelControl.getHeading(), intakeTunnel.getHeading())
                .addPath(new BezierCurve(intakeTunnel, gateControl, shootWithBack))
                .setLinearHeadingInterpolation(intakeTunnel.getHeading(), shootWithBack.getHeading())
                .build();


        shootSpike1 = follower.pathBuilder()
                .addPath(new BezierLine(shootWithBack, spike1Start))
                .setLinearHeadingInterpolation(shootWithBack.getHeading(), spike1Start.getHeading())
                .addPath(new BezierLine(spike1Start, spike1End))
                .setLinearHeadingInterpolation(spike1Start.getHeading(), spike1End.getHeading())
                .addPath(new BezierLine(spike1End, shootWithBack))
                .setLinearHeadingInterpolation(spike1End.getHeading(), shootWithBack.getHeading())
                .build();

        shootToEnd = follower.pathBuilder()
                .addPath(new BezierLine(shootWithBack, endPose))
                .setLinearHeadingInterpolation(shootWithBack.getHeading(), endPose.getHeading())
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
        return AllianceColor.RED;
    }
}
