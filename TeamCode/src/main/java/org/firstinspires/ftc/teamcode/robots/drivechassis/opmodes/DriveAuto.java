package org.firstinspires.ftc.teamcode.robots.drivechassis.opmodes;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.robots.base.opmodes.BaseOpMode;
import org.firstinspires.ftc.teamcode.robots.drivechassis.DriveGamepadHandler;
import org.firstinspires.ftc.teamcode.robots.drivechassis.DriveRobot;

@Autonomous(name = "Drive Auto", preselectTeleOp = "Drive")
public class DriveAuto extends BaseOpMode<DriveRobot, DriveGamepadHandler, DriveGamepadHandler> {
    Pose startPose = new Pose(0, 0, 0);
    Pose secondPose = new Pose(36, 12, 0);


    @Override
    public void init() {
        super.init();
    }

    @Override
    public void start() {
        PathChain path = robot.getFollower().pathBuilder()
                .addPath(new BezierLine(startPose, secondPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), secondPose.getHeading())
                .setTangentHeadingInterpolation()
                .build();

        robot.getFollower().followPath(path);
    }

    @Override
    protected DriveRobot instantiateRobot() {
        return new DriveRobot();
    }

    @Override
    protected Pose instantiateStartPose() {
        return null;
    }

    @Override
    protected DriveGamepadHandler instantiateGamepadHandler1() {
        return new DriveGamepadHandler(robot, this, gamepad1, true);
    }

    @Override
    protected DriveGamepadHandler instantiateGamepadHandler2() {
        return new DriveGamepadHandler(robot, this, gamepad1, true);
    }
}
