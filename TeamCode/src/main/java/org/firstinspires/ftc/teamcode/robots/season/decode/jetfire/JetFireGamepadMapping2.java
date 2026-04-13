package org.firstinspires.ftc.teamcode.robots.season.decode.jetfire;

import static org.firstinspires.ftc.teamcode.robots.base.StaticData.allianceColor;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.hardware.controllers.servo.RGBIndicatorLightController;
import org.firstinspires.ftc.teamcode.robots.base.GamepadMapping;
import org.firstinspires.ftc.teamcode.robots.base.opmodes.OpModeBase;

public class JetFireGamepadMapping2 extends GamepadMapping<JetfireRobot> {
    double zoneOffsetIncrement = 1;
    public static final double smallZoneOffsetIncrement = 1;
    public static final double largeZoneOffsetIncrement = 5;


    public JetFireGamepadMapping2(JetfireRobot jetfireRobot, Gamepad gamepad) {
        super(jetfireRobot, gamepad);
    }

    @Override
    public void onYPressed() {
        robot.zeroTurntable();
    }

    @Override
    public void onBPressed() {

    }

    @Override
    public void onAPressed() {
        robot.humanPlayerPoseReset();
    }

    @Override
    public void onXPressed() {
        robot.toggleMuzzleFlash();
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

    }

    @Override
    public void onDpadUpPressed() {
        zoneOffsetIncrement = largeZoneOffsetIncrement;
    }

    @Override
    public void onDpadRightPressed() {
        robot.adjustActiveTurntableZoneOffset(-zoneOffsetIncrement);
    }

    @Override
    public void onDpadDownPressed() {
        zoneOffsetIncrement = smallZoneOffsetIncrement;
    }

    @Override
    public void onDpadLeftPressed() {
        robot.adjustActiveTurntableZoneOffset(zoneOffsetIncrement);
    }

    @Override
    public void joysticks(float leftX, float leftY, float rightX, float rightY) {

    }

    @Override
    public void onLeftStickPressed() {

    }

    @Override
    public void onRightStickPressed() {

    }
}
