package org.firstinspires.ftc.teamcode.robots.season.decode.jetfire;

import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.teamcode.robots.base.PrimaryDriverGamepadMapping;

public class JetfireGamepadMapping extends PrimaryDriverGamepadMapping<JetfireRobot> {
    public JetfireGamepadMapping(JetfireRobot jetfireRobot, Gamepad gamepad) {
        super(jetfireRobot, gamepad);
    }

    @Override
    public void onYPressed() {
        robot.setFlywheelOn(!robot.isFlywheelOn());
    }

    @Override
    public void onBPressed() {

    }

    @Override
    public void onAPressed() {
        robot.toggleSubsystems(!robot.isFlywheelOn());
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
        robot.setReverseIntake(val > 0.05 ? true : false);
    }

    @Override
    public void rightTrigger(float val) {
         if (val > 0.05 && robot.isReadyToShoot()){
             robot.rapidFire();
             gamepad.rumble(200);
         }
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
        robot.setIntakeOn(!robot.isIntakeOn());
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

    }

    @Override
    public void onDpadDownPressed() {

    }

    @Override
    public void onDpadLeftPressed() {

    }
}
