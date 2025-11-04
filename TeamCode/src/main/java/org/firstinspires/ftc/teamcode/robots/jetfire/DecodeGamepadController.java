package org.firstinspires.ftc.teamcode.robots.jetfire;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.robots.base.GamepadHandlerBase;
import org.firstinspires.ftc.teamcode.robots.jetfire.opmodes.teleops.Teleop;

public class DecodeGamepadController extends GamepadHandlerBase<DecodeRobot, Teleop> {
    private boolean isIntakeRunning = false;
    private boolean areFlywheelsActive = false;

    private boolean wasRightTriggerPressed = false;
    private boolean wasLeftTriggerPressed = false;

    public DecodeGamepadController(DecodeRobot decodeRobot, Teleop opMode, Gamepad gamepad, boolean allowDrive) {
        super(decodeRobot, opMode, gamepad, allowDrive);
    }

    @Override
    protected void onProcessGamepadControls(Gamepad gamepad) {
        // FLYWHEELS
//        if (gamepad.rightBumperWasPressed()) {
//            if (areFlywheelsActive) {
//                robot.flywheel1.setRPS(0);
//                robot.flywheel2.setRPS(0);
//            } else {
//                robot.flywheel1.setRPS(15);
//                robot.flywheel2.setRPS(30);
//            }
//            areFlywheelsActive = !areFlywheelsActive;
//        }

        // LAUNCH
//        if (gamepad.right_trigger > 0.1) {
//            if (!wasRightTriggerPressed) {
//                robot.launchArtifact();
//            }
//            wasRightTriggerPressed = true;
//        } else {
//            wasRightTriggerPressed = false;
//        }


        // INTAKE
//        if (gamepad.leftBumperWasPressed()) {
//            isIntakeRunning = !isIntakeRunning;
//        }

//        if (gamepad.left_trigger > 0.1) {
//            robot.reverseIntake();
//        } else {
//            if (isIntakeRunning) {
//                robot.startIntake();
//            } else {
//                robot.stopIntake();
//            }
//        }
//
//        if (gamepad.xWasPressed()) {
//            robot.flicker();
//        }


//        if (gamepad.yWasPressed()) {
//            Pose currentPose = robot.getFollower().getPose();
//            Pose goalPose = new Pose(-55, 55);
//            Pose diff = goalPose.minus(currentPose);
//
//            PathChain path = robot.getFollower().pathBuilder()
//                    .addPath(new BezierPoint(currentPose.getX(), currentPose.getY()))
//                    .setHeadingConstraint(Math.atan2(diff.getY(), diff.getX()))
//                    .build();
//
//            robot.getFollower().followPath(path);
//
//        }




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