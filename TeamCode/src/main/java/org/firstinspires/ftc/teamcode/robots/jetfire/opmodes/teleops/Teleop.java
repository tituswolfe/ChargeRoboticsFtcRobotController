package org.firstinspires.ftc.teamcode.robots.jetfire.opmodes.teleops;


import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robots.base.GamepadMapping;
import org.firstinspires.ftc.teamcode.robots.base.opmodes.TeleOpBase;
import org.firstinspires.ftc.teamcode.robots.jetfire.DecodeRobot;

@TeleOp(name = "Decode")
public class Teleop extends TeleOpBase<DecodeRobot> {

    @Override
    public void loop() {
        telemetry.addLine("- flywheel -");
        telemetry.addData("top target RPS", robot.flywheel1.getConfiguredRPS());
        telemetry.addData("bottom target RPS", robot.flywheel2.getConfiguredRPS());
        telemetry.addLine();
        telemetry.addData("top actual RPS", robot.flywheel1.getCurrentRPS());
        telemetry.addData("bottom actual RPS", robot.flywheel2.getCurrentRPS());
        telemetry.addLine();
        telemetry.addData("top pow", robot.flywheel1.getDcMotorEx().getPower());
        telemetry.addData("bottom pow", robot.flywheel2.getDcMotorEx().getPower());

//        graphManager.addData("top RPS", robot.flywheel1.getCurrentRPS());
//        graphManager.addData("bottom RPS", robot.flywheel2.getCurrentRPS());
//        graphManager.update();
        panelsTelemetry.getTelemetry().addData("top RPS", robot.flywheel1.getCurrentRPS());
        panelsTelemetry.getTelemetry().addData("bottom RPS", robot.flywheel2.getCurrentRPS());
        panelsTelemetry.getTelemetry().update(telemetry);

        super.loop();
    }

    @Override
    public void start() {
        robot.getFollower().startTeleopDrive();
        robot.getFollower().update();
        super.start();
    }

    @Override
    protected DecodeRobot instantiateRobot() {
        return new DecodeRobot();
    }

    @Override
    protected Pose instantiateStartPose() {
        return null;
    }

    @Override
    protected GamepadMapping instantiateGamepadMapping1() {
        return null;
    }

    @Override
    protected GamepadMapping instantiateGamepadMapping2() {
        return null;
    }

    @Override
    protected AllianceColor instantiateAllianceColor() {
        return AllianceColor.RED;
    }
}
