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
        telemetry.addLine("- FLYWHEEL -");
        telemetry.addData("TOP_RPM", robot.getTopFlywheel().getVelocity());
        telemetry.addData("BOTTOM_RPM", robot.getBottomFlywheel().getVelocity());
        telemetry.addLine("");
        telemetry.addData("TOP_TARGET_RPM", robot.getTopFlywheel().getTargetVelocity());
        telemetry.addData("BOTTOM_TARGET_RPM", robot.getBottomFlywheel().getTargetVelocity());
        telemetry.addLine("");
        telemetry.addData("TOP_POWER", robot.getTopFlywheel().getDcMotorEx().getPower());
        telemetry.addData("BOTTOM_POWER", robot.getBottomFlywheel().getDcMotorEx().getPower());
        telemetry.addLine("");
        telemetry.addData("TOP_FLYWHEEL_RATIO", robot.topFlywheelRatio);
        telemetry.addLine("");

        telemetry.addLine("- INTAKE -");
        telemetry.addData("INTAKE_POWER", robot.getIntake().getDcMotorEx().getPower());
        telemetry.addLine("");



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
