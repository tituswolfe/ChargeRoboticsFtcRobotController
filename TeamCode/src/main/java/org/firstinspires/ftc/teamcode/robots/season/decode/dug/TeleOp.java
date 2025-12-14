package org.firstinspires.ftc.teamcode.robots.season.decode.dug;

import com.pedropathing.follower.Follower;

import org.firstinspires.ftc.teamcode.robots.base.GamepadMapping;
import org.firstinspires.ftc.teamcode.robots.base.opmodes.TeleOpBase;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "JetFire", group = "jetfire")
public class TeleOp extends TeleOpBase<JetFireRobot> {
    @Override
    public void buildPaths(Follower follower) {

    }

    @Override
    protected JetFireRobot instantiateRobot() {
        return new JetFireRobot(); // TODO: In super?
    }

    @Override
    protected GamepadMapping<JetFireRobot> instantiateGamepadMapping1() {
        return new DugGamepadMapping(robot);
    }

    @Override
    protected GamepadMapping<JetFireRobot> instantiateGamepadMapping2() {
        return null;
    }
}
