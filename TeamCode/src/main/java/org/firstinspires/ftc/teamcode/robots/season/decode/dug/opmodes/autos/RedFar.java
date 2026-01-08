package org.firstinspires.ftc.teamcode.robots.season.decode.dug.opmodes.autos;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.robots.base.opmodes.BaseAuto;
import org.firstinspires.ftc.teamcode.robots.season.decode.dug.DugRobot;


@Autonomous(name = "Red Far", group = "dug", preselectTeleOp = "Dug")
@Disabled
public class RedFar extends BaseAuto<DugRobot> {
    public Pose startPose = new Pose(57.8, 24, Math.toRadians(-180));
    public Pose endPose = new Pose(33.8, 24, Math.toRadians(-180));

    public PathChain moveOffLine;

    @Override
    public void autonomousPathUpdate(int pathState) {
        switch (pathState) {
            case 0:
                robot.getFollower().followPath(moveOffLine, true);
                setPathState(-1, true);
                break;
            case -1:
                break;
        }
    }

    @Override
    public void buildPaths(Follower follower) {
        moveOffLine = follower.pathBuilder()
                .addPath(new BezierLine(startPose, endPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), endPose.getHeading())
                .build();
    }

    @Override
    protected DugRobot instantiateRobot() {
        return new DugRobot();
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
