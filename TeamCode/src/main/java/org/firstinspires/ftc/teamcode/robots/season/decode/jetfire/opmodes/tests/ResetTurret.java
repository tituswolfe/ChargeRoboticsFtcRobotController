package org.firstinspires.ftc.teamcode.robots.season.decode.jetfire.opmodes.tests;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robots.base.GamepadMapping;
import org.firstinspires.ftc.teamcode.robots.base.opmodes.TeleOpBase;
import org.firstinspires.ftc.teamcode.robots.season.decode.jetfire.JetfireRobot;
import org.firstinspires.ftc.teamcode.robots.season.decode.jetfire.JetfireStaticData;
import org.firstinspires.ftc.teamcode.util.math.Angle;

@TeleOp(name="Reset Turret")
public class ResetTurret extends TeleOpBase<JetfireRobot> {

    @Override
    public void start() {
        JetfireStaticData.lastTurretHeading = new Angle(0);
        stop(); // TODO: Bad practice?
        super.start();
    }

    @Override
    public void buildPaths(Follower follower) {

    }

    @Override
    protected JetfireRobot instantiateRobot() {
        return null;
    }

    @Override
    protected GamepadMapping<JetfireRobot> instantiateGamepadMapping1() {
        return null;
    }

    @Override
    protected GamepadMapping<JetfireRobot> instantiateGamepadMapping2() {
        return null;
    }
}
