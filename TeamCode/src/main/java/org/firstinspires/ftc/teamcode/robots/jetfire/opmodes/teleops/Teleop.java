package org.firstinspires.ftc.teamcode.robots.jetfire.opmodes.teleops;


import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robots.base.GamepadMapping;
import org.firstinspires.ftc.teamcode.robots.base.opmodes.TeleOpBase;
import org.firstinspires.ftc.teamcode.robots.jetfire.JetFireRobot;
import org.firstinspires.ftc.teamcode.robots.jetfire.JetFireGamepadMapping;

@TeleOp(name = "Decode")
public class Teleop extends TeleOpBase<JetFireRobot> {
    @Override
    public void updateTelemetry(TelemetryManager telemetry) {
        telemetry.addLine("- flywheel -");
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
    protected GamepadMapping instantiateGamepadMapping1() {
        return new JetFireGamepadMapping(robot);
    }

    @Override
    protected GamepadMapping instantiateGamepadMapping2() {
        return null;
    }
}
