package org.firstinspires.ftc.teamcode.robots.season.decode.jetfire;

import static org.firstinspires.ftc.teamcode.robots.base.StaticData.allianceColor;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.hardware.controllers.servo.RGBIndicatorLightController;
import org.firstinspires.ftc.teamcode.robots.base.GamepadMapping;
import org.firstinspires.ftc.teamcode.robots.base.opmodes.OpModeBase;

public class JetFireGamepadMapping2 extends GamepadMapping<JetfireRobot> {
    double zoneOffsetIncrement = 1;

    public JetFireGamepadMapping2(JetfireRobot jetfireRobot, Gamepad gamepad) {
        super(jetfireRobot, gamepad);
    }

    @Override
    public void onYPressed() {
        robot.humanPlayerPoseReset();
    }

    @Override
    public void onBPressed() {
        robot.getTurret().turntableController().setInitialAngle(0);
    }

    @Override
    public void onAPressed() {
        if (allianceColor.equals(OpModeBase.AllianceColor.BLUE)) {
            robot.closeTurntableOffsetDeg = JetfireRobot.CLOSE_ZONE_TURNTABLE_START_OFFSET_BLUE;
            robot.farTurntableOffsetDeg = JetfireRobot.FAR_ZONE_TURNTABLE_START_OFFSET_BLUE;
        } else {
            robot.closeTurntableOffsetDeg = JetfireRobot.CLOSE_ZONE_TURNTABLE_START_OFFSET_RED;
            robot.farTurntableOffsetDeg = JetfireRobot.FAR_ZONE_TURNTABLE_START_OFFSET_RED;
        }
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
        if (robot.isInFarZone()) {
            robot.farTurntableOffsetDeg += zoneOffsetIncrement;
        } else {
            robot.closeTurntableOffsetDeg += zoneOffsetIncrement;
        }
    }

    @Override
    public void onRightBumperPressed() {
        if (robot.isInFarZone()) {
            robot.farTurntableOffsetDeg -= zoneOffsetIncrement;
        } else {
            robot.closeTurntableOffsetDeg -= zoneOffsetIncrement;
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
