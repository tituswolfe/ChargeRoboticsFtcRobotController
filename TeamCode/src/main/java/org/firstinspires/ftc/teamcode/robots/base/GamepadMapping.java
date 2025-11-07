package org.firstinspires.ftc.teamcode.robots.base;

import com.qualcomm.robotcore.hardware.Gamepad;

public abstract class GamepadMapping<Robot extends RobotBase> {
    protected final Robot robot;
    boolean wasLeftTriggerPressed = false;
    boolean wasRightTriggerPressed = false;

    public GamepadMapping(Robot robot) {
        this.robot = robot;
    }

    // Buttons
    public abstract void onYPressed();
    public abstract void onBPressed();
    public abstract void onAPressed();
    public abstract void onXPressed();

    // TODO: On released
    // TODO: While held

    // Joysticks
    public abstract void leftJoystick(float x, float y);
    public abstract void rightJoystick(float x, float y);
    public void joysticks(float leftX, float leftY, float rightX, float rightY) {
        if (robot.getFollower() == null) return;
        if (!robot.getFollower().isTeleopDrive()) return;

        if (robot.isRobotCentric) {
            robot.getFollower().setTeleOpDrive(leftY * robot.speedFactor, leftX * robot.speedFactor, rightX * robot.speedFactor, true);
        } else {
            robot.getFollower().setTeleOpDrive(leftY * robot.speedFactor, leftX * robot.speedFactor, rightX * robot.speedFactor, false, robot.fieldCentricOffset);
        }
    };

    public void onLeftStickPressed() {
        robot.isRobotCentric = !robot.isRobotCentric;
    }

    public void onRightStickPressed() {
        robot.isSlowMode = !robot.isSlowMode;
        if (robot.isSlowMode) {
            robot.speedFactor = RobotBase.slowSpeedFactor;
        } else {
            robot.speedFactor = 1;
        }
    }

    // Triggers
    public abstract void leftTrigger(float val);
    public abstract void rightTrigger(float val);

    public abstract void onLeftTriggerPressed();
    public abstract void onRightTriggerPressed();

    public abstract void onLeftTriggerReleased();
    public abstract void onRightTriggerReleased();

    // Bumper
    public abstract void onLeftBumperPressed();
    public abstract void onRightBumperPressed();

    // D-pad
    public abstract void onDpadUpPressed();
    public abstract void onDpadRightPressed();
    public abstract void onDpadDownPressed();
    public abstract void onDpadLeftPressed();

    public void processGamepad(Gamepad gamepad) {
        boolean isLeftTriggerPressed = gamepad.right_trigger > 0.1;
        boolean isRightTriggerPressed = gamepad.right_trigger > 0.1;

        // Buttons
        if (gamepad.yWasPressed()) onYPressed();
        if (gamepad.bWasPressed()) onBPressed();
        if (gamepad.aWasPressed()) onAPressed();
        if (gamepad.xWasPressed()) onXPressed();

        // Joysticks
        leftJoystick(-gamepad.left_stick_x, -gamepad.left_stick_y);
        rightJoystick(-gamepad.right_stick_x, -gamepad.right_stick_y);
        joysticks(-gamepad.left_stick_x, -gamepad.left_stick_y, -gamepad.right_stick_x, -gamepad.right_stick_y);

        if (gamepad.leftStickButtonWasPressed()) onLeftStickPressed();
        if (gamepad.rightStickButtonWasPressed()) onRightStickPressed();

        // Trigger
        leftTrigger(gamepad.left_trigger);
        rightTrigger(gamepad.right_trigger);

        if (isLeftTriggerPressed && !wasLeftTriggerPressed) onLeftTriggerPressed();
        if (!isLeftTriggerPressed && wasLeftTriggerPressed) onLeftTriggerReleased();

        if (isRightTriggerPressed && !wasRightTriggerPressed) onRightTriggerPressed();
        if (!isRightTriggerPressed && wasRightTriggerPressed) onRightTriggerReleased();

        // Bumper
        if (gamepad.leftBumperWasPressed()) onLeftBumperPressed();
        if (gamepad.rightBumperWasPressed()) onRightBumperPressed();

        if (gamepad.dpadUpWasPressed()) onDpadUpPressed();
        if (gamepad.dpadRightWasPressed()) onDpadRightPressed();
        if (gamepad.dpadDownWasPressed()) onDpadDownPressed();
        if (gamepad.dpadLeftWasPressed()) onDpadLeftPressed();


        wasLeftTriggerPressed = isLeftTriggerPressed;
        wasRightTriggerPressed = isRightTriggerPressed;
    }

    public boolean toggle(boolean state, Runnable onTrue, Runnable onFalse) {
        if (state) {
            onFalse.run();
        } else {
            onTrue.run();
        }

        return !state;
    }
}

// JOYSTICK INPUT
//           | <---- y axis
//           |
//         -1.0
//           /\
//           ___
//         / \|/ \
// -1.0 < |--( )--| > 1.0 ---- x axis
//         \ /|\ /
//           ---
//           \/
//           1.0

// RIGHT STICK 0.0 to -1.0
// LEFT STICK 0.0 to 1.0