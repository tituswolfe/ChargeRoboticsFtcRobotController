package org.firstinspires.ftc.teamcode.robots.season.decode.jetfire.opmodes.autos.worlds;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.robots.base.opmodes.BaseAuto;
import org.firstinspires.ftc.teamcode.robots.season.decode.jetfire.JetfireRobot;
import org.firstinspires.ftc.teamcode.util.actionsequence.Action;
import org.firstinspires.ftc.teamcode.util.actionsequence.ActionSequence;
import org.firstinspires.ftc.teamcode.util.actionsequence.InstantAction;
import org.firstinspires.ftc.teamcode.util.actionsequence.PathAction;

import java.util.ArrayList;

@Autonomous(name = "CHARGERS (BLUE+CLOSE)", preselectTeleOp = "Jetfire")
public class BlueClose extends BaseAuto<JetfireRobot> {
    public static Pose startPose = new Pose();
    Pose shootPose = new Pose();


    public static Pose endPose = new Pose();

    PathChain startToShoot;

    PathChain shootToSpike2;
    PathChain spike2ToShoot;

    PathChain shootToGate;
    PathChain gateToShoot;

    PathChain shootToSpike1;
    PathChain spike1ToShoot;

    PathChain shootToEnd;

    int gateCycleAttempts = 2;

    ActionSequence shootPreload = new ActionSequence(new Action[] {
            new PathAction(startToShoot, robot.getFollower()),
            new InstantAction(() -> robot.fire())
    });

    ActionSequence shootSpike2 = new ActionSequence(new Action[] {
            new PathAction(shootToSpike2, robot.getFollower()),
            new PathAction(spike2ToShoot, robot.getFollower()),
            new InstantAction(() -> robot.fire())
    });

    ActionSequence shootGateCycle = new ActionSequence(new Action[] {
            new PathAction(shootToGate, robot.getFollower()),
            new PathAction(gateToShoot, robot.getFollower()),
            new InstantAction(() -> robot.fire())
    });

    ActionSequence shootSpike1 = new ActionSequence(new Action[] {
            new PathAction(shootToSpike1, robot.getFollower()),
            new PathAction(spike1ToShoot, robot.getFollower()),
            new InstantAction(() -> robot.fire())
    });

    @Override
    public void autonomousPathUpdate(int pathState) {
        switch (pathState) {
            case 0:
                robot.toggleSubsystems(true);
                shootPreload.start();
                nextPathState();
                break;
            case 1:
                if (!shootPreload.isRunning()) {
                    shootSpike2.start();
                    nextPathState();
                }
                break;
            case 2:
                if (!shootSpike2.isRunning()) {
                    shootSpike2.start();
                    nextPathState();
                }
                break;
            case 3:
                if (!shootSpike2.isRunning() && !shootGateCycle.isRunning()) {
                    shootGateCycle.start();
                    nextPathState();
                }
                break;
            case 4:
                if (gateCycleAttempts <= 0) {
                    setPathState(3);
                    gateCycleAttempts--;
                } else {
                    setPathState(5);
                }
                break;
            case 5:
                if (!shootGateCycle.isRunning()) {
                    shootSpike1.start();
                    nextPathState();
                }
                break;
            case 6:
                if (!shootSpike1.isRunning()) {
                    robot.getFollower().followPath(shootToEnd);
                    setPathState(-1);
                }
                break;
        }
    }

    @Override
    public ArrayList<Pose> generatePoses() {
        ArrayList<Pose> poses = new ArrayList();

        poses.add(new Pose());



        return poses;
    }


    @Override
    public void buildPaths(Follower follower) {
        shootPose.mirror();
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
