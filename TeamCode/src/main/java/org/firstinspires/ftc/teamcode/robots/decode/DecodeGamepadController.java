package org.firstinspires.ftc.teamcode.robots.decode;

import static java.lang.Thread.sleep;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.robots.base.GamepadHandlerBase;
import org.firstinspires.ftc.teamcode.robots.decode.opmodes.teleops.Teleop;

public class DecodeGamepadController extends GamepadHandlerBase<DecodeRobot, Teleop> {
    private boolean isIntakeRunning = false;
    private boolean areFlywheelsActive = false;


    public DecodeGamepadController(DecodeRobot decodeRobot, Teleop opMode, Gamepad gamepad, boolean allowDrive) {
        super(decodeRobot, opMode, gamepad, allowDrive);
    }

    @Override
    protected void onProcessGamepadControls(Gamepad gamepad) {
        if (gamepad.yWasPressed()) {
            isIntakeRunning = !isIntakeRunning;
            if (isIntakeRunning) {
                robot.runIntake();
            } else {
                robot.stopIntake();
            }
        }

        if (gamepad.aWasPressed()) {
            Pose startPose = robot.getFollower().getPose();
            Pose targetPose = new Pose(38.8, 33, 0);

            PathChain path = robot.getFollower().pathBuilder()
                    .addPath(new BezierLine(startPose, targetPose))
                    .setLinearHeadingInterpolation(startPose.getHeading(), targetPose.getHeading())
                    .build();

            robot.getFollower().followPath(path);
        }

        if(gamepad.rightBumperWasPressed()) {
            gamepad.rumble(150);
            robot.launchArtifact();
        }
        //robot.lanchServo.setPosition((-gamepad.right_trigger * (0.7 - 0.27)) + 0.27);

        if (gamepad.bWasPressed()) {
            if (areFlywheelsActive) {
                robot.flywheel1.setRPS(0);
                robot.flywheel2.setRPS(0);
            } else {
                robot.flywheel1.setRPS(20);
                robot.flywheel2.setRPS(35);
            }
            areFlywheelsActive = !areFlywheelsActive;
        }

        if(gamepad.dpadUpWasPressed()) {
            robot.flywheel1.setRPS(robot.flywheel1.getConfiguredRPS() + 1);
            lockOutGamepad();
        }

        if(gamepad.dpadDownWasPressed()) {
            robot.flywheel1.setRPS(robot.flywheel1.getConfiguredRPS() - 1);
            lockOutGamepad();
        }

        if(gamepad.dpadRightWasPressed()) {
            robot.flywheel2.setRPS(robot.flywheel2.getConfiguredRPS() + 1);
            lockOutGamepad();
        }

        if(gamepad.dpadLeftWasPressed()) {
            robot.flywheel2.setRPS(robot.flywheel2.getConfiguredRPS() - 1);
            lockOutGamepad();
        }
    }
}