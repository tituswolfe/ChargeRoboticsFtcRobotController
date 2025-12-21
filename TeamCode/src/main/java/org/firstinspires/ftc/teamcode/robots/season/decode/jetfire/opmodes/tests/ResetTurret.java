package org.firstinspires.ftc.teamcode.robots.season.decode.jetfire.opmodes.tests;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robots.base.GamepadMapping;
import org.firstinspires.ftc.teamcode.robots.base.opmodes.TeleOpBase;
import org.firstinspires.ftc.teamcode.robots.season.decode.jetfire.JetFireRobot;
import org.firstinspires.ftc.teamcode.robots.season.decode.jetfire.JetFireStaticData;
import org.firstinspires.ftc.teamcode.util.math.Angle;

@TeleOp(name="Reset Turret")
public class ResetTurret extends TeleOpBase<JetFireRobot> {

    @Override
    public void start() {
        JetFireStaticData.lastTurretHeading = new Angle(0);
        stop(); // TODO: Bad practice?
        super.start();
    }

    @Override
    public void buildPaths(Follower follower) {

    }

    @Override
    protected JetFireRobot instantiateRobot() {
        return null;
    }

    @Override
    protected GamepadMapping<JetFireRobot> instantiateGamepadMapping1() {
        return null;
    }

    @Override
    protected GamepadMapping<JetFireRobot> instantiateGamepadMapping2() {
        return null;
    }
}
