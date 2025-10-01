package org.firstinspires.ftc.teamcode.robots.drivechassis.opmodes;

import static org.firstinspires.ftc.teamcode.hardware.drivetrain.pedroPathing.Tuning.follower;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robots.base.opmodes.BaseOpMode;
import org.firstinspires.ftc.teamcode.robots.drivechassis.DriveGamepadHandler;
import org.firstinspires.ftc.teamcode.robots.drivechassis.DriveRobot;

@TeleOp(name = "Drive")
public class DriveTeleOp extends BaseOpMode<DriveRobot, DriveGamepadHandler, DriveGamepadHandler> {

    @Override
    public void init() {
        super.init();
    }

    @Override
    protected void adjustHardwareBeforeStart() {

    }

    @Override
    public void start() {
        robot.getFollower().startTeleopDrive();
        robot.getFollower().update();
    }

    @Override
    public void loop() {
        super.loop();
    }

    @Override
    protected DriveRobot instantiateRobot() {
        return new DriveRobot();
    }

    @Override
    protected Pose instantiateStartPose() {
        return null;
    }

    @Override
    protected DriveGamepadHandler instantiateGamepadHandler1() {
        return new DriveGamepadHandler(robot, this, gamepad1, true);
    }

    @Override
    protected DriveGamepadHandler instantiateGamepadHandler2() {
        return new DriveGamepadHandler(robot, this, gamepad2, true);
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
