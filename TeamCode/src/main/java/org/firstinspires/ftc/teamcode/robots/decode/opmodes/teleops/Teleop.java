package org.firstinspires.ftc.teamcode.robots.decode.opmodes.teleops;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robots.base.opmodes.TeleOpBase;
import org.firstinspires.ftc.teamcode.robots.decode.DecodeRobot;
import org.firstinspires.ftc.teamcode.robots.decode.DecodeGamepadController;


@TeleOp(name = "Decode")
public class Teleop extends TeleOpBase<DecodeRobot, DecodeGamepadController, DecodeGamepadController> {
    @Override
    public void loop() {
        telemetry.addLine("- flywheel -");
        telemetry.addData("top set RPS", robot.flywheel1.getConfiguredRPS());
        telemetry.addData("bottom set RPS", robot.flywheel2.getConfiguredRPS());
        telemetry.addLine();
        telemetry.addData("top actual RPS", robot.flywheel1.getCurrentRPS());
        telemetry.addData("bottom actual RPS", robot.flywheel2.getCurrentRPS());
        telemetry.addLine();
        telemetry.addData("top pow", robot.flywheel1.getDcMotorEx().getPower());
        telemetry.addData("bottom pow", robot.flywheel2.getDcMotorEx().getPower());

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
    protected DecodeGamepadController instantiateGamepadHandler1() {
        return new DecodeGamepadController(robot, this, gamepad1, true);
    }

    @Override
    protected DecodeGamepadController instantiateGamepadHandler2() {
        return new DecodeGamepadController(robot, this, gamepad2, false);
    }

    @Override
    protected AllianceColor instantiateAllianceColor() {
        return AllianceColor.RED;
    }
}
