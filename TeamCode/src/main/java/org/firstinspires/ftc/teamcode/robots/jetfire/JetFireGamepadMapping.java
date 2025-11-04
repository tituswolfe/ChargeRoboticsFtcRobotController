package org.firstinspires.ftc.teamcode.robots.jetfire;

import org.firstinspires.ftc.teamcode.robots.base.GamepadMapping;

public class JetFireGamepadMapping extends GamepadMapping<DecodeRobot> {
    boolean isIntakeActive = false;
    boolean areFlywheelsActive = false;

    public JetFireGamepadMapping(DecodeRobot decodeRobot) {
        super(decodeRobot);
    }

    @Override
    public void onYPressed() {
        robot.flicker();
    }

    @Override
    public void onBPressed() {

    }

    @Override
    public void onAPressed() {

    }

    @Override
    public void onXPressed() {
        // TODO: Rotate toward goal
    }

    @Override
    public void leftJoystick(float x, float y) {

    }

    @Override
    public void rightJoystick(float x, float y) {

    }

    @Override
    public void joysticks(float leftX, float leftY, float rightX, float rightY) {

    }

    @Override
    public void leftTrigger(float val) {
        if (val > 0.1) {
            robot.reverseIntake();
        } else if (isIntakeActive) {
            robot.startIntake();
        }
    }

    @Override
    public void rightTrigger(float val) {

    }

    @Override
    public void onLeftTriggerPressed() {

    }

    @Override
    public void onRightTriggerPressed() {
        robot.launchArtifact();

    }

    @Override
    public void onLeftTriggerReleased() {

    }

    @Override
    public void onRightTriggerReleased() {

    }

    @Override
    public void onLeftBumperPressed() {
        isIntakeActive = !isIntakeActive;
        if (isIntakeActive) {
            robot.startIntake();
        } else {
            robot.stopIntake();
        }
    }

    @Override
    public void onRightBumperPressed() {
        areFlywheelsActive = !areFlywheelsActive;
        if (areFlywheelsActive) {
            robot.startFlywheels();
        } else {
            robot.stopFlywheels();
        }
    }

    @Override
    public void onDpadUpPressed() {

    }

    @Override
    public void onDpadRightPressed() {

    }

    @Override
    public void onDpadDownPressed() {

    }

    @Override
    public void onDpadLeftPressed() {

    }
}
