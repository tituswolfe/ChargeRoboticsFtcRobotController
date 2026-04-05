package org.firstinspires.ftc.teamcode.robots.season.decode.jetfire;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.hardware.controllers.servo.RGBIndicatorLightController;
import org.firstinspires.ftc.teamcode.robots.base.GamepadMapping;
import org.firstinspires.ftc.teamcode.robots.base.StaticData;
import org.firstinspires.ftc.teamcode.robots.base.opmodes.OpModeBase;
import org.firstinspires.ftc.teamcode.util.math.Angle;

public class JetFireGamepadMapping2 extends GamepadMapping<JetfireRobot> {
    public JetFireGamepadMapping2(JetfireRobot jetfireRobot, Gamepad gamepad) {
        super(jetfireRobot, gamepad);
    }

    @Override
    public void onYPressed() {
        robot.getFollower().setPose(JetfireRobot.humanPlayerReset);
        robot.getIndicatorLightController().indicate(RGBIndicatorLightController.Color.VIOLET);
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

    double zoneOffsetIncrement = 1;

    @Override
    public void onLeftBumperPressed() {
        if (robot.isInFarZone()) {
            JetfireRobot.FAR_ZONE_TURNTABLE_OFFSET_DEG += zoneOffsetIncrement;
        } else {
            JetfireRobot.CLOSE_ZONE_TURNTABLE_OFFSET_DEG += zoneOffsetIncrement;
        }
    }

    @Override
    public void onRightBumperPressed() {
        if (robot.isInFarZone()) {
            JetfireRobot.FAR_ZONE_TURNTABLE_OFFSET_DEG -= zoneOffsetIncrement;
        } else {
            JetfireRobot.CLOSE_ZONE_TURNTABLE_OFFSET_DEG -= zoneOffsetIncrement;
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

    @Override
    public void joysticks(float leftX, float leftY, float rightX, float rightY) {
        // super.joysticks(leftX, leftY, rightX, rightY);
    }

    @Override
    public void onLeftStickPressed() {

    }

    @Override
    public void onRightStickPressed() {
        //super.onRightStickPressed();
    }
}
