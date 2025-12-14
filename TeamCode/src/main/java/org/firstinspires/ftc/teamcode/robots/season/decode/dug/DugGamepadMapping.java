package org.firstinspires.ftc.teamcode.robots.season.decode.dug;

import org.firstinspires.ftc.teamcode.robots.base.GamepadMapping;

public class DugGamepadMapping extends GamepadMapping<JetFireRobot> {
    public DugGamepadMapping(JetFireRobot dugRobot) {
        super(dugRobot);
    }

    @Override
    public void onYPressed() {

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
