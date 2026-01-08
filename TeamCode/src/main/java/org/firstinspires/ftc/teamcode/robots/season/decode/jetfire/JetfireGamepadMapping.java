package org.firstinspires.ftc.teamcode.robots.season.decode.jetfire;

import org.firstinspires.ftc.teamcode.robots.base.GamepadMapping;

public class JetfireGamepadMapping extends GamepadMapping<JetfireRobot> {
    public JetfireGamepadMapping(JetfireRobot dugRobot) {
        super(dugRobot);
    }

    @Override
    public void onYPressed() {
        robot.setFlywheelMode(switch (robot.getFlywheelMode()) {
            case AUTO -> JetfireRobot.FlywheelMode.OFF;
            case OFF -> JetfireRobot.FlywheelMode.AUTO;
        });
    }

    @Override
    public void onBPressed() {

    }

    @Override
    public void onAPressed() {
        robot.startTurret();
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
         if (val > 0.1 && robot.isReadyToShoot()) robot.shoot();
    }

    @Override
    public void onLeftTriggerPressed() {
        robot.setReverseIntake(true);
    }

    @Override
    public void onRightTriggerPressed() {

    }

    @Override
    public void onLeftTriggerReleased() {
        robot.setReverseIntake(false);
    }

    @Override
    public void onRightTriggerReleased() {

    }

    @Override
    public void onLeftBumperPressed() {
        robot.setIntakeMode(switch (robot.getIntakeMode()) {
            case ON -> JetfireRobot.IntakeMode.OFF;
            case OFF -> JetfireRobot.IntakeMode.ON;
        });
    }

    @Override
    public void onRightBumperPressed() {
        robot.setAutoAimTurntable(!robot.isAutoAimTurntable());
    }

    @Override
    public void onDpadUpPressed() {

    }

    @Override
    public void onDpadRightPressed() {
        robot.setDriverTurntableOffset(robot.getDriverTurntableOffset() - 1);
    }

    @Override
    public void onDpadDownPressed() {

    }

    @Override
    public void onDpadLeftPressed() {
        robot.setDriverTurntableOffset(robot.getDriverTurntableOffset() + 1);
    }
}
