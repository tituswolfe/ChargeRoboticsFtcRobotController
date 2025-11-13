package org.firstinspires.ftc.teamcode.robots.season.decode.jetfire.opmodes.teleops;

import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robots.base.GamepadMapping;
import org.firstinspires.ftc.teamcode.robots.base.opmodes.TeleOpBase;
import org.firstinspires.ftc.teamcode.robots.season.decode.jetfire.JetFireRobot;
import org.firstinspires.ftc.teamcode.robots.season.decode.jetfire.JetFireGamepadMapping;


@TeleOp(name = "JetFire")
public class Teleop extends TeleOpBase<JetFireRobot> {
    @Override
    public void buildPaths(Follower follower) {

    }

    @Override
    public void start() {
        robot.startConfiguration();
        super.start();
    }

    @Override
    public void updateTelemetry(TelemetryManager telemetry) {
        telemetry.addLine("- FLYWHEEL DATA -");
        telemetry.addData("Top Flywheel RPS", robot.getTopFlywheel().getCurrentRPS());
        telemetry.addData("Bottom Flywheel RPS", robot.getBottomFlywheel().getCurrentRPS());
        telemetry.addLine("");
        telemetry.addData("top target RPS", robot.getTopFlywheel().getConfiguredRPS());
        telemetry.addData("bottom target RPS", robot.getBottomFlywheel().getConfiguredRPS());
        telemetry.addLine("");
        telemetry.addData("top pow", robot.getTopFlywheel().getDcMotorEx().getPower());
        telemetry.addData("bottom pow", robot.getBottomFlywheel().getDcMotorEx().getPower());


        super.updateTelemetry(telemetry);
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
        return null; // new JetFireGamepadMapping2(robot);
    }
}
