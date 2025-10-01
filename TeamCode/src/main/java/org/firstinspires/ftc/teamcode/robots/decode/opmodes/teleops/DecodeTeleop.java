package org.firstinspires.ftc.teamcode.robots.decode.opmodes.teleops;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robots.base.opmodes.BaseOpMode;
import org.firstinspires.ftc.teamcode.robots.decode.DecodeRobot;
import org.firstinspires.ftc.teamcode.robots.decode.NealsonGamepadController;


@TeleOp(name = "Decode")
public class DecodeTeleop extends BaseOpMode<DecodeRobot, NealsonGamepadController, NealsonGamepadController> {
    @Override
    protected void adjustHardwareBeforeStart() {

    }

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
    protected DecodeRobot instantiateRobot() {
        return new DecodeRobot();
    }

    @Override
    protected Pose instantiateStartPose() {
        return null;
    }

    @Override
    protected NealsonGamepadController instantiateGamepadHandler1() {
        return new NealsonGamepadController(robot, this, gamepad1, true);
    }

    @Override
    protected NealsonGamepadController instantiateGamepadHandler2() {
        return new NealsonGamepadController(robot, this, gamepad2, false);
    }

    @Override
    protected AlliancePosition instantiateAlliancePosition() {
        return null;
    }

    @Override
    protected FieldType instantiateFieldType() {
        return null;
    }
}
