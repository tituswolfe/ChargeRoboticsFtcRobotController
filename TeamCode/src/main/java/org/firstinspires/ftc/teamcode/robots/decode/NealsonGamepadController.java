package org.firstinspires.ftc.teamcode.robots.decode;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.robots.base.GamepadHandlerBase;
import org.firstinspires.ftc.teamcode.robots.decode.opmodes.teleops.DecodeTeleop;
import org.firstinspires.ftc.teamcode.util.ThreadUtil;

public class NealsonGamepadController extends GamepadHandlerBase<DecodeRobot, DecodeTeleop> {
    private boolean isIntakeRunning = false;


    public NealsonGamepadController(DecodeRobot decodeRobot, DecodeTeleop opMode, Gamepad gamepad, boolean allowDrive) {
        super(decodeRobot, opMode, gamepad, allowDrive);
    }

    @Override
    protected void onProcessGamepadControls(Gamepad gamepad) {
        if (gamepad.aWasPressed()) {
            isIntakeRunning = !isIntakeRunning;
            if (isIntakeRunning) {
                robot.intake.setRPS(15);
            } else {
                robot.intake.setRPS(0);
            }
        }

//        if (gamepad.left_trigger > 0.1) {
//            robot.intake.setRPS(15);
//        } else {
//            robot.intake.setRPS(0);
//        }

        if(gamepad.bWasPressed()) {
            robot.servo.setPosition(0.7);
            ThreadUtil.sleep(500);
            robot.servo.setPosition(0.27);
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

//    private void processDrivingInput(Gamepad gamepad) {
//        if (gamepad.rightStickButtonWasPressed()) {
//            isFastMode = !isFastMode;
//            if (isFastMode) {
//                currentDriveSpeedMultiplier = driveSpeedMultiplierFastSpeed;
//                currentTurnSpeedMultiplier = turnSpeedMultiplierFastSpeed;
//            } else {
//                currentDriveSpeedMultiplier = driveSpeedMultiplierNormalSpeed;
//                currentTurnSpeedMultiplier = turnSpeedMultiplierNormalSpeed;
//            }
//        }
//
//        double x_speed = (gamepad.left_stick_x > minStickSensitivity ? gamepad.left_stick_x * currentDriveSpeedMultiplier : 0) + (gamepad.dpad_left ? -0.2 : 0) + (gamepad.dpad_right ? 0.2 : 0);
//        double y_speed = (gamepad.left_stick_y > minStickSensitivity ? gamepad.left_stick_y * currentDriveSpeedMultiplier : 0) + (gamepad.dpad_up ? 0.2 : 0) + (gamepad.dpad_down ? -0.2 : 0);
//        double r_speed = gamepad.right_stick_x > minStickSensitivity ? gamepad.right_stick_x * currentTurnSpeedMultiplier : 0;
//
//        //opMode.getRobot().get().setTeleOpDrive(y_speed, x_speed, r_speed, false);
//
////        double cosHeading = opMode.getRobot().getDriveTrain().getCosHeading();
////        double sinHeading = opMode.getRobot().getDriveTrain().getSinHeading();
////
////        opMode.getRobot().getDriveTrain().setMotorPowersThroughKinematicTransformation(
////                opMode.getRobot().getDriveTrain().calculateRobotStrafeFromFieldVelocity(x_speed, cosHeading, y_speed, sinHeading),
////                opMode.getRobot().getDriveTrain().calculateRobotForwardFromFieldVelocity(x_speed, cosHeading, y_speed, sinHeading),
////                r_speed
////        );