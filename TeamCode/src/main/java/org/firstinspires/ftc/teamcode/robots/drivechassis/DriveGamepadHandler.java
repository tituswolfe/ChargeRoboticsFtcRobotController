package org.firstinspires.ftc.teamcode.robots.drivechassis;

import static org.firstinspires.ftc.teamcode.hardware.drivetrain.pedroPathing.Tuning.follower;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.robots.base.GamepadHandlerBase;
import org.firstinspires.ftc.teamcode.robots.base.RobotBase;
import org.firstinspires.ftc.teamcode.robots.base.opmodes.BaseOpMode;

public class DriveGamepadHandler extends GamepadHandlerBase {
    public DriveGamepadHandler(RobotBase robotBase, BaseOpMode opMode, Gamepad gamepad, boolean allowDrive) {
        super(robotBase, opMode, gamepad, allowDrive);
    }

    @Override
    protected void onProcessGamepadControls(Gamepad gamepad) {
        if (gamepad.aWasPressed()) {
            Pose startPose = robot.getFollower().getPose();
            Pose targetPose = new Pose(35.6, 32.7, 0);

            PathChain path = robot.getFollower().pathBuilder()
                    .addPath(new BezierLine(startPose, targetPose))
                    .setLinearHeadingInterpolation(startPose.getHeading(), targetPose.getHeading())
                    .setTangentHeadingInterpolation()
                    .build();

            robot.getFollower().followPath(path);
        }

        robot.getFollower().setTeleOpDrive(-gamepad.left_stick_y, -gamepad.left_stick_x, -gamepad.right_stick_x, true);
    }
}
