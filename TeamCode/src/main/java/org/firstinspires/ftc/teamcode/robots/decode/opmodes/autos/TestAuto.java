package org.firstinspires.ftc.teamcode.robots.decode.opmodes.autos;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.robots.base.GamepadHandlerBase;
import org.firstinspires.ftc.teamcode.robots.base.RobotBase;
import org.firstinspires.ftc.teamcode.robots.base.opmodes.BaseOpMode;
import org.firstinspires.ftc.teamcode.robots.decode.DecodeRobot;

public class TestAuto extends BaseOpMode {
    @Override
    protected void adjustHardwareBeforeStart() {

    }

    @Override
    protected void buildPaths() {

    }

    @Override
    protected void autonomousPathUpdate(int pathState) {

    }

    @Override
    protected RobotBase instantiateRobot() {
        return new DecodeRobot();
    }

    @Override
    protected Pose instantiateStartPose() {
        return null;
    }

    @Override
    protected GamepadHandlerBase instantiateGamepadHandler1() {
        return null;
    }

    @Override
    protected GamepadHandlerBase instantiateGamepadHandler2() {
        return null;
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
