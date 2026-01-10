package org.firstinspires.ftc.teamcode.robots.season.decode.jetfire.opmodes.autos;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.robots.base.opmodes.BaseAuto;
import org.firstinspires.ftc.teamcode.robots.season.decode.jetfire.JetfireRobot;

@Autonomous(preselectTeleOp = "Jetfire")
public class RedFar extends BaseAuto<JetfireRobot> {
    Pose startPose = new Pose(62.7, 12.7, Math.toRadians(180));
    Pose endPose = new Pose(35.5, 21.8, Math.toRadians(90));

    PathChain endPath;

    int returnPathState = 0;

    @Override
    public void autonomousPathUpdate(int pathState) {
        switch (pathState) {
            case 0:
                returnPathState = 1;
                setPathState(1, true);
                break;
            case 1:
                if (!robot.getFollower().isBusy() && actionTimer.getElapsedTimeSeconds() > 5) {
                    robot.getFollower().followPath(endPath);
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
        endPath = follower.pathBuilder()
                .addPath(new BezierLine(startPose, endPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), endPose.getHeading())
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
