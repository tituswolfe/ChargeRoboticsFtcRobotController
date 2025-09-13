package org.firstinspires.ftc.teamcode.robots.standard;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.robots.base.GamepadControllerBase;

@Deprecated
public abstract class StandardGamepadController {
    private boolean allowGamepad2DriverControls;

    public static double driveSpeedMultiplierNormalSpeed = 0.75;
    public static double turnSpeedMultiplierNormalSpeed = 0.6;

    public static double driveSpeedMultiplierFastSpeed = 1;
    public static double turnSpeedMultiplierFastSpeed = 0.75;

    private double currentDriveSpeedMultiplier = driveSpeedMultiplierNormalSpeed;
    private double currentTurnSpeedMultiplier = turnSpeedMultiplierNormalSpeed;

    public static double minStickSensitivity = 0.05;

    private boolean isFastMode = false;

//    public StandardGamepadController(StandardOpMode opMode, Gamepad gamepad1, Gamepad gamepad2, boolean allowGamepad2DriverControls) {
//        super(opMode, gamepad1, gamepad2);
//        this.allowGamepad2DriverControls = allowGamepad2DriverControls;
//    }
//
//    @Override
//    public void processGamepadControls() {
//        processDrivingInput(gamepad1);
//        if (allowGamepad2DriverControls) {
//            processDrivingInput(gamepad2);
//        }
//        super.processGamepadControls();
//    }


    private void processDrivingInput(Gamepad gamepad) {
        if (gamepad.rightStickButtonWasPressed()) {
            isFastMode = !isFastMode;
            if (isFastMode) {
                currentDriveSpeedMultiplier = driveSpeedMultiplierFastSpeed;
                currentTurnSpeedMultiplier = turnSpeedMultiplierFastSpeed;
            } else {
                currentDriveSpeedMultiplier = driveSpeedMultiplierNormalSpeed;
                currentTurnSpeedMultiplier = turnSpeedMultiplierNormalSpeed;
            }
        }

        double x_speed = (gamepad.left_stick_x > minStickSensitivity ? gamepad.left_stick_x * currentDriveSpeedMultiplier : 0) + (gamepad.dpad_left ? -0.2 : 0) + (gamepad.dpad_right ? 0.2 : 0);
        double y_speed = (gamepad.left_stick_y > minStickSensitivity ? gamepad.left_stick_y * currentDriveSpeedMultiplier : 0) + (gamepad.dpad_up ? 0.2 : 0) + (gamepad.dpad_down ? -0.2 : 0);
        double r_speed = gamepad.right_stick_x > minStickSensitivity ? gamepad.right_stick_x * currentTurnSpeedMultiplier : 0;

        //opMode.getRobot().get().setTeleOpDrive(y_speed, x_speed, r_speed, false);

//        double cosHeading = opMode.getRobot().getDriveTrain().getCosHeading();
//        double sinHeading = opMode.getRobot().getDriveTrain().getSinHeading();
//
//        opMode.getRobot().getDriveTrain().setMotorPowersThroughKinematicTransformation(
//                opMode.getRobot().getDriveTrain().calculateRobotStrafeFromFieldVelocity(x_speed, cosHeading, y_speed, sinHeading),
//                opMode.getRobot().getDriveTrain().calculateRobotForwardFromFieldVelocity(x_speed, cosHeading, y_speed, sinHeading),
//                r_speed
//        );
    }
}
