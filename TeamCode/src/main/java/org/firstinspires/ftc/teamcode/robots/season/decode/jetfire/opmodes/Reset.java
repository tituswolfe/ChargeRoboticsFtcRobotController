package org.firstinspires.ftc.teamcode.robots.season.decode.jetfire.opmodes;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robots.base.GamepadMapping;
import org.firstinspires.ftc.teamcode.robots.base.opmodes.OpModeBase;
import org.firstinspires.ftc.teamcode.robots.season.decode.jetfire.JetfireRobot;
import org.firstinspires.ftc.teamcode.robots.season.decode.jetfire.JetfireStaticData;
import org.firstinspires.ftc.teamcode.util.math.Angle;

@TeleOp(name = "RESET")
public class Reset extends OpModeBase<JetfireRobot> {

    @Override
    public void start() {
        super.start();

        JetfireStaticData.lastTurretHeading = new Angle(0);
        robot.getFollower().setPose(new Pose(0, 0, 0));

        requestOpModeStop();
    }

    @Override
    public void opModeTypeSpecificInit() {

    }

    @Override
    public void buildPaths(Follower follower) {

    }

    @Override
    protected JetfireRobot instantiateRobot() {
        return new JetfireRobot();
    }

    @Override
    protected Pose instantiateStartPose() {
        return null;
    }

    @Override
    protected AllianceColor instantiateAllianceColor() {
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
