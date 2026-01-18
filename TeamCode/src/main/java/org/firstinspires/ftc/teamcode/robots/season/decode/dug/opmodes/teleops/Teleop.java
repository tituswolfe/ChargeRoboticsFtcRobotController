package org.firstinspires.ftc.teamcode.robots.season.decode.dug.opmodes.teleops;

import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robots.base.GamepadMapping;
import org.firstinspires.ftc.teamcode.robots.base.opmodes.TeleOpBase;
import org.firstinspires.ftc.teamcode.robots.season.decode.dug.DugRobot;
import org.firstinspires.ftc.teamcode.robots.season.decode.dug.DugGamepadMapping;


@TeleOp(name = "Dug", group = "dug")
@Disabled
public class Teleop extends TeleOpBase<DugRobot> {
    @Override
    public void buildPaths(Follower follower) {

    }

    @Override
    public void start() {
        robot.startConfiguration();
        super.start();
    }
//
//    @Override
//    public void updateTelemetry(TelemetryManager telemetry) {
//        telemetry.addLine("- FLYWHEEL -");
//        telemetry.addData("TOP_RPM", robot.getTopFlywheel().getVelocity());
//        telemetry.addData("BOTTOM_RPM", robot.getBottomFlywheel().getVelocity());
//        telemetry.addLine("");
//        telemetry.addData("TOP_TARGET_RPM", robot.getTopFlywheel().getTargetVelocity());
//        telemetry.addData("BOTTOM_TARGET_RPM", robot.getBottomFlywheel().getTargetVelocity());
//        telemetry.addLine("");
//        telemetry.addData("TOP_POWER", robot.getTopFlywheel().getDcMotorEx().getPower());
//        telemetry.addData("BOTTOM_POWER", robot.getBottomFlywheel().getDcMotorEx().getPower());
//        telemetry.addLine("");
//        telemetry.addData("TOP_FLYWHEEL_RATIO", robot.topFlywheelRatio);
//        telemetry.addLine("");
//        telemetry.addData("Goal Heading Offset", robot.headingGoalOffsetDeg);
//
//
//        super.updateTelemetry(telemetry);
//    }

    @Override
    protected DugRobot instantiateRobot() {
        return new DugRobot();
    }

    @Override
    protected GamepadMapping<DugRobot> instantiateGamepadMapping1() {
        return new DugGamepadMapping(robot);
    }

    @Override
    protected GamepadMapping<DugRobot> instantiateGamepadMapping2() {
        return null; // new JetFireGamepadMapping2(robot);
    }
}
