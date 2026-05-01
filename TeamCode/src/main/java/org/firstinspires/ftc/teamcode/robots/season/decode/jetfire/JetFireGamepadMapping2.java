package org.firstinspires.ftc.teamcode.robots.season.decode.jetfire;

import static org.firstinspires.ftc.teamcode.robots.base.StaticData.allianceColor;

import com.bylazar.gamepad.GamepadManager;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.hardware.controllers.servo.RGBIndicatorLightController;
import org.firstinspires.ftc.teamcode.robots.base.GamepadMapping;
import org.firstinspires.ftc.teamcode.robots.base.opmodes.OpModeBase;

public class JetFireGamepadMapping2 extends GamepadMapping<JetfireRobot> {
    double zoneOffsetIncrement = 1;

    public JetFireGamepadMapping2(JetfireRobot jetfireRobot, Gamepad gamepad, GamepadManager virtualGamepad) {
        super(jetfireRobot, gamepad, virtualGamepad);
    }


    @Override
    public void onYPressed() {
        robot.resetOffsets();
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
        //robot.toggleMuzzleFlash();
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
        robot.adjustActiveTurntableZoneOffset(JetFireConstants.TURNTABLE_ZONE_OFFSET_INCREMENT);

    }

    @Override
    public void onRightBumperPressed() {
        robot.adjustActiveTurntableZoneOffset(-JetFireConstants.TURNTABLE_ZONE_OFFSET_INCREMENT);

    }

    @Override
    public void onDpadUpPressed() {
        robot.adjustFlywheelTrim(JetFireConstants.FLYWHEEL_OFFSET_INCREMENT);
    }

    @Override
    public void onDpadRightPressed() {
        robot.adjustActiveTurntableZoneOffset(-JetFireConstants.TURNTABLE_ZONE_OFFSET_INCREMENT);
    }

    @Override
    public void onDpadDownPressed() {
        robot.adjustFlywheelTrim(-JetFireConstants.FLYWHEEL_OFFSET_INCREMENT);
    }

    @Override
    public void onDpadLeftPressed() {
        robot.adjustActiveTurntableZoneOffset(JetFireConstants.TURNTABLE_ZONE_OFFSET_INCREMENT);
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
