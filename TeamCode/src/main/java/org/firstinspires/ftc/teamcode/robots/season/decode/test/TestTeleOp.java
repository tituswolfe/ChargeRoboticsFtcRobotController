package org.firstinspires.ftc.teamcode.robots.season.decode.test;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.robots.base.GamepadMapping;
import org.firstinspires.ftc.teamcode.robots.base.opmodes.TeleOpBase;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Test Bot", group = "jetfire")
@Disabled
public class TestTeleOp extends TeleOpBase<TestRobot> {
    @Override
    public void buildPaths(Follower follower) {

    }

    @Override
    protected TestRobot instantiateRobot() {
        return new TestRobot();
    }

    @Override
    protected GamepadMapping<TestRobot> instantiateGamepadMapping1() {
        return new TestMapping(robot, gamepad1);
    }

    @Override
    protected GamepadMapping<TestRobot> instantiateGamepadMapping2() {
        return null;
    }

}
