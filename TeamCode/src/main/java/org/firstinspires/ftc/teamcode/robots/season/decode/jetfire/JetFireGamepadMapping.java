package org.firstinspires.ftc.teamcode.robots.season.decode.jetfire;

import org.firstinspires.ftc.teamcode.robots.base.GamepadMapping;

public class JetFireGamepadMapping extends GamepadMapping<JetFireRobot> {
    public JetFireGamepadMapping(JetFireRobot dugRobot) {
        super(dugRobot);
    }

    @Override
    public void onYPressed() {
        robot.setFlywheelMode(switch (robot.getFlywheelMode()) {
            case AUTO -> null;
            case SET_VELOCITY -> JetFireRobot.FlywheelMode.OFF;
            case OFF -> JetFireRobot.FlywheelMode.SET_VELOCITY;
        });
    }

    @Override
    public void onBPressed() {

    }

    @Override
    public void onAPressed() {

    }

    @Override
    public void onXPressed() {
    }

    @Override
    public void leftJoystick(float x, float y) {

    }

    @Override
    public void rightJoystick(float x, float y) {

    }

    @Override
    public void leftTrigger(float val) {

    }

    @Override
    public void rightTrigger(float val) {
        // if (val > 0.1) robot.shoot();
    }

    @Override
    public void onLeftTriggerPressed() {

    }

    @Override
    public void onRightTriggerPressed() {

    }

    @Override
    public void onLeftTriggerReleased() {

    }

    @Override
    public void onRightTriggerReleased() {

    }

    @Override
    public void onLeftBumperPressed() {
        robot.setIntakeMode(switch (robot.getIntakeMode()) {
            case INTAKE -> JetFireRobot.IntakeMode.OFF;
            case REVERSE -> JetFireRobot.IntakeMode.OFF;
            case OFF -> JetFireRobot.IntakeMode.INTAKE;
        });
    }

    @Override
    public void onRightBumperPressed() {
        robot.setAutoAimOn(!robot.isAutoAimOn());
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
