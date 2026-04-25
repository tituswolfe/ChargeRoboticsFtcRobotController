package org.firstinspires.ftc.teamcode.robots.season.decode.jetfire.opmodes.teleops;

import com.bylazar.gamepad.PanelsGamepad;
import com.pedropathing.follower.Follower;

import org.firstinspires.ftc.teamcode.robots.base.GamepadMapping;
import org.firstinspires.ftc.teamcode.robots.base.opmodes.TeleOpBase;
import org.firstinspires.ftc.teamcode.robots.season.decode.jetfire.JetFireGamepadMapping2;
import org.firstinspires.ftc.teamcode.robots.season.decode.jetfire.JetfireGamepadMapping;
import org.firstinspires.ftc.teamcode.robots.season.decode.jetfire.JetfireRobot;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Jetfire", group = "jetfire")
public class TeleOp extends TeleOpBase<JetfireRobot> {
    @Override
    public void buildPaths(Follower follower) {

    }

    @Override
    protected JetfireRobot instantiateRobot() {
        return new JetfireRobot();
    }

    @Override
    protected GamepadMapping<JetfireRobot> instantiateGamepadMapping1() {
        //PanelsGamepad.INSTANCE.getFirstManager().asCombinedFTCGamepad(gamepad1);
        return new JetfireGamepadMapping(robot, gamepad1, null);
    }

    @Override
    protected GamepadMapping<JetfireRobot> instantiateGamepadMapping2() {
        //PanelsGamepad.INSTANCE.getFirstManager().asCombinedFTCGamepad(gamepad2);
        return new JetFireGamepadMapping2(robot, gamepad2,  null);
    }
}
