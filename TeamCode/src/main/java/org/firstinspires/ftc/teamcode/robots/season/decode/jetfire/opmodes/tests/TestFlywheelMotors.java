package org.firstinspires.ftc.teamcode.robots.season.decode.jetfire.opmodes.tests;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.robots.base.opmodes.BaseAuto;
import org.firstinspires.ftc.teamcode.robots.season.decode.jetfire.JetfireRobot;

import java.util.ArrayList;

@Autonomous(name = "Test Flywheel Motors", group = "test")
public class TestFlywheelMotors extends BaseAuto<JetfireRobot> {

    @Override
    public void autonomousPathUpdate(int pathState) {
        switch (pathState) {
            case 0:
                robot.getTurret().flywheelController().getDevice().setPower(0.2);
                nextPathState();
                break;
            case 1:
                if (actionTimer.getElapsedTime() > 1000) {
                    robot.getTurret().flywheelController().getDevice().setPower(0);
                    nextPathState();
                }
                break;
            case 2:
                robot.getTurret().flywheelController().getSecondMotor().setPower(-0.2);
                setPathState(3);
                break;
            case 3:
                if (actionTimer.getElapsedTime() > 1000) {
                    robot.getTurret().flywheelController().getSecondMotor().setPower(0);
                    setPathState(-1);
                }
                break;
        }
    }

    @Override
    public ArrayList<Pose> generatePoses() {
        return null;
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
        return new Pose();
    }

    @Override
    protected AllianceColor instantiateAllianceColor() {
        return AllianceColor.BLUE;
    }
}
