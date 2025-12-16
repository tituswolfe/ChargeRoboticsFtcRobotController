package org.firstinspires.ftc.teamcode.robots.season.decode.jetfire.opmodes.teleops;

import com.pedropathing.follower.Follower;

import org.firstinspires.ftc.teamcode.robots.base.GamepadMapping;
import org.firstinspires.ftc.teamcode.robots.base.opmodes.TeleOpBase;
import org.firstinspires.ftc.teamcode.robots.season.decode.jetfire.JetFireGamepadMapping;
import org.firstinspires.ftc.teamcode.robots.season.decode.jetfire.JetFireRobot;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "JetFire", group = "jetfire")
public class TeleOp extends TeleOpBase<JetFireRobot> {
    @Override
    public void buildPaths(Follower follower) {

    }

    @Override
    protected JetFireRobot instantiateRobot() {
        return new JetFireRobot();
    }

    @Override
    protected GamepadMapping<JetFireRobot> instantiateGamepadMapping1() {
        return new JetFireGamepadMapping(robot);
    }

    @Override
    protected GamepadMapping<JetFireRobot> instantiateGamepadMapping2() {
        return null;
    }
}
