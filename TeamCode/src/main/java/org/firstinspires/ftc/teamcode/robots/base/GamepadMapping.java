package org.firstinspires.ftc.teamcode.robots.base;

import com.qualcomm.robotcore.hardware.Gamepad;

public abstract class GamepadMapping<Robot extends RobotBase> {
    protected final Robot robot;
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
    public abstract void joysticks(float leftX, float leftY, float rightX, float rightY);

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

    boolean wasLeftTriggerPressed = false;
    boolean wasRightTriggerPressed = false;

    // TODO: Make static method
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

        // Trigger
        leftTrigger(gamepad.left_trigger);
        rightTrigger(gamepad.right_trigger);

        if (isLeftTriggerPressed && !wasLeftTriggerPressed) onLeftTriggerPressed();
        if (!isLeftTriggerPressed && wasLeftTriggerPressed) onLeftTriggerReleased();

        if (isRightTriggerPressed && !wasRightTriggerPressed) onRightTriggerPressed();
        if (!isRightTriggerPressed && wasRightTriggerPressed) onRightTriggerReleased();

        // Bumper
        onLeftBumperPressed();

        if (gamepad.dpadUpWasPressed());
        if (gamepad.dpadRightWasPressed());
        if (gamepad.dpadDownWasPressed());
        if (gamepad.dpadLeftWasPressed());


        wasLeftTriggerPressed = isLeftTriggerPressed;
        wasRightTriggerPressed = isRightTriggerPressed;
    }
}
