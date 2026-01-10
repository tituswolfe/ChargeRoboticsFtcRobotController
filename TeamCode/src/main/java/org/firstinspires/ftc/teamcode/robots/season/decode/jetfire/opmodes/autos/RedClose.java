package org.firstinspires.ftc.teamcode.robots.season.decode.jetfire.opmodes.autos;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.robots.base.opmodes.BaseAuto;
import org.firstinspires.ftc.teamcode.robots.season.decode.jetfire.JetfireRobot;

@Autonomous(preselectTeleOp = "Jetfire")
public class RedClose extends BaseAuto<JetfireRobot> {
    Pose startPose = new Pose(-57.24, 43.5, Math.toRadians(127.5));
    Pose shootPose = new Pose(-14.5, 18, Math.toRadians(90));
    Pose intakeLine1 = new Pose(-14.5, 50.5, Math.toRadians(90));

    PathChain shootPath;
    PathChain intakePath;

    int returnPathState = 0;

    @Override
    public void autonomousPathUpdate(int pathState) {
        switch (pathState) {
            case 0:
                robot.getFollower().followPath(shootPath);
                //robot.getAutoFire().start();
                returnPathState = 1;
                setPathState(1, true);
                break;
            case 1:
                if (!robot.getFollower().isBusy() && actionTimer.getElapsedTimeSeconds() > 5) {
                    robot.getFollower().followPath(intakePath);
                    setPathState(2, true);
                }
                break;
            case 2:
                if (!robot.getFollower().isBusy()) {

                    setPathState(-1, true);
                }
                break;



            case 20:
                if (robot.isReadyToShoot()) {
                    robot.fire();
                    setPathState(21, true);
                }
                break;
            case 21:
                if (robot.isReadyToShoot() && actionTimer.getElapsedTime() > 250) {
                    robot.fire();
                    setPathState(22, true);
                }
                break;
            case 22:
                if (robot.isReadyToShoot()&& actionTimer.getElapsedTime() > 250) {
                    robot.fire();
                    setPathState(returnPathState, true);
                }
                break;
        }
    }

    @Override
    public void buildPaths(Follower follower) {
        shootPath = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();

        intakePath = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, intakeLine1))
                .setLinearHeadingInterpolation(shootPose.getHeading(), intakeLine1.getHeading())
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
