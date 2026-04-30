package org.firstinspires.ftc.teamcode.robots.season.decode.jetfire;

import com.bylazar.gamepad.GamepadManager;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.teamcode.robots.base.PrimaryDriverGamepadMapping;

public class JetfireGamepadMapping extends PrimaryDriverGamepadMapping<JetfireRobot> {
    double zoneOffsetIncrement = 1;


    public JetfireGamepadMapping(JetfireRobot jetfireRobot, Gamepad gamepad, GamepadManager virtualGamepad) {
        super(jetfireRobot, gamepad, virtualGamepad);
    }


    @Override
    public void onYPressed() {
        robot.setFlywheelOn(!robot.isFlywheelOn());
    }

    @Override
    public void onBPressed() {
        robot.setAutoAimTurntable(!robot.isAutoAimTurntable());
    }

    @Override
    public void onAPressed() {
        robot.toggleSubsystems(!robot.isFlywheelOn());
    }

    @Override
    public void onXPressed() {
        robot.setIntakeOn(!robot.isIntakeOn());
    }

    @Override
    public void leftJoystick(float x, float y) {

    }

    @Override
    public void rightJoystick(float x, float y) {

    }

    @Override
    public void leftTrigger(float val) {
        robot.setReverseIntake(val > 0.05);
    }

    @Override
    public void rightTrigger(float val) {

    }

    @Override
    public void onLeftTriggerPressed() {

    }

    @Override
    public void onRightTriggerPressed() {
        robot.fire();
    }

    @Override
    public void onLeftTriggerReleased() {

    }

    @Override
    public void onRightTriggerReleased() {

    }

    @Override
    public void onLeftBumperPressed() {
        robot.adjustActiveTurntableZoneOffset(zoneOffsetIncrement);
        //robot.setIntakeOn(!robot.isIntakeOn());
    }

    @Override
    public void onRightBumperPressed() {
        robot.adjustActiveTurntableZoneOffset(-zoneOffsetIncrement);
        //robot.setAutoAimTurntable(!robot.isAutoAimTurntable());
    }

    @Override
    public void onDpadUpPressed() {

    }

    @Override
    public void onDpadRightPressed() {
        robot.adjustActiveTurntableZoneOffset(-zoneOffsetIncrement);
    }

    @Override
    public void onDpadDownPressed() {

    }

    @Override
    public void onDpadLeftPressed() {
        robot.adjustActiveTurntableZoneOffset(zoneOffsetIncrement);
    }
}
