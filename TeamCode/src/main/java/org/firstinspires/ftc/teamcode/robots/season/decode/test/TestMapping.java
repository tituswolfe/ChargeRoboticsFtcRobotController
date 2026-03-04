package org.firstinspires.ftc.teamcode.robots.season.decode.test;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.robots.base.PrimaryDriverGamepadMapping;
import org.firstinspires.ftc.teamcode.robots.base.StaticData;
import org.firstinspires.ftc.teamcode.robots.base.opmodes.OpModeBase;

public class TestMapping extends PrimaryDriverGamepadMapping<TestRobot> {

    public TestMapping(TestRobot testRobot, Gamepad gamepad) {
        super(testRobot, gamepad);
    }

    @Override
    public void onYPressed() {
        robot.setFieldCentricOffset(StaticData.allianceColor == OpModeBase.AllianceColor.BLUE ? OpModeBase.AllianceColor.RED : OpModeBase.AllianceColor.BLUE);
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
