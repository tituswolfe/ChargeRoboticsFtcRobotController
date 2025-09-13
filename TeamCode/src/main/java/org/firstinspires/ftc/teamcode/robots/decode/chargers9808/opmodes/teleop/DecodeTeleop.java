package org.firstinspires.ftc.teamcode.robots.Decode.chargers9808.opmodes.teleop;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.drawCurrentAndHistory;
import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.CoordinateSystem;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robots.Decode.chargers9808.DecodeRobot9808;
import org.firstinspires.ftc.teamcode.robots.base.BaseOpMode;
import org.firstinspires.ftc.teamcode.robots.base.GamepadControllerBase;

@TeleOp(name = "Decode Teleop")
public class DecodeTeleOp extends BaseOpMode<DecodeRobot9808, GamepadControllerBase> {

    @Override
    public void init() {
        setupOpMode(new DecodeRobot9808());
        super.init();
    }

    Pose startPose = new Pose(0, 0, 0);
    Pose secondPose = new Pose(36, 12, 0);

    @Override
    public void start() {
        robot.getFollower().activateDrive();
        robot.getFollower().activateAllPIDFs();
        PathChain path = robot.getFollower().pathBuilder()
                .addPath(new BezierLine(startPose, secondPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), secondPose.getHeading())
                .setTangentHeadingInterpolation()
                .build();


        robot.getFollower().followPath(path);
    }

    @Override
    public void loop() {
        robot.getFollower().update();
        super.loop();
    }
}
