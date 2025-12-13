package org.firstinspires.ftc.teamcode.robots.season.decode.dug;

import com.pedropathing.follower.Follower;

import org.firstinspires.ftc.teamcode.robots.base.GamepadMapping;
import org.firstinspires.ftc.teamcode.robots.base.opmodes.TeleOpBase;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Dug temp", group = "jetfire")
public class TeleOp extends TeleOpBase<DugRobot> {
    @Override
    public void buildPaths(Follower follower) {

    }

    @Override
    protected DugRobot instantiateRobot() {
        return new DugRobot(); // TODO: In super?
    }

    @Override
    protected GamepadMapping<DugRobot> instantiateGamepadMapping1() {
        return new DugGamepadMapping(robot);
    }

    @Override
    protected GamepadMapping<DugRobot> instantiateGamepadMapping2() {
        return null;
    }
}
