package org.firstinspires.ftc.teamcode.robots.jetfire;

import com.pedropathing.geometry.BezierPoint;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.robots.base.GamepadMapping;
import org.firstinspires.ftc.teamcode.robots.base.StaticData;
import org.firstinspires.ftc.teamcode.robots.base.opmodes.OpModeBase;

public class JetFireGamepadMapping extends GamepadMapping<JetFireRobot> {
    boolean isIntakeActive = false;

    public JetFireGamepadMapping(JetFireRobot decodeRobot) {
        super(decodeRobot);
    }

    @Override
    public void onYPressed() {
        robot.flicker();
    }

    @Override
    public void onBPressed() {

    }

    @Override
    public void onAPressed() {
        // robot.getTopFlywheel().getDcMotorEx().setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(JetFireRobot.p, JetFireRobot.i, JetFireRobot.d, JetFireRobot.f));
        // robot.getBottomFlywheel().getDcMotorEx().setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(JetFireRobot.p, JetFireRobot.i, JetFireRobot.d, JetFireRobot.f));
    }

    @Override
    public void onXPressed() {
        Pose currentPose = robot.getFollower().getPose();

        if (StaticData.allianceColor == OpModeBase.AllianceColor.RED) {
            robot.getFollower().turnTo(Math.atan2(robot.redGoal.getY() - currentPose.getY(), robot.redGoal.getX() - currentPose.getX()));
        } else {
            robot.getFollower().turnTo(Math.atan2(robot.redGoal.mirror().getY() - currentPose.getY(), robot.redGoal.mirror().getX() - currentPose.getX()));
        }
    }

    @Override
    public void leftJoystick(float x, float y) {

    }

    @Override
    public void rightJoystick(float x, float y) {

    }

    @Override
    public void leftTrigger(float val) {
        if (val > 0.1) {
            robot.reverseIntake();
        } else if (isIntakeActive) {
            robot.startIntake();
        } else {
            robot.stopIntake();
        }
    }

    @Override
    public void rightTrigger(float val) {

    }

    @Override
    public void onLeftTriggerPressed() {

    }

    @Override
    public void onRightTriggerPressed() {
        robot.launchArtifact();

    }

    @Override
    public void onLeftTriggerReleased() {

    }

    @Override
    public void onRightTriggerReleased() {

    }

    @Override
    public void onLeftBumperPressed() {
        isIntakeActive = !isIntakeActive;
        if (isIntakeActive) {
            robot.startIntake();
        } else {
            robot.stopIntake();
        }
        // isIntakeActive = toggle(isIntakeActive, robot::startIntake, robot::stopIntake);
    }

    @Override
    public void onRightBumperPressed() {
        robot.toggleFlywheelPreset();
    }

    @Override
    public void onDpadUpPressed() {
        robot.getTopFlywheel().setRPS(robot.getTopFlywheel().getConfiguredRPS() + 1);
    }

    @Override
    public void onDpadRightPressed() {
        robot.getBottomFlywheel().setRPS(robot.getBottomFlywheel().getConfiguredRPS() + 1);
    }

    @Override
    public void onDpadDownPressed() {
        robot.getTopFlywheel().setRPS(robot.getTopFlywheel().getConfiguredRPS() - 1);

    }

    @Override
    public void onDpadLeftPressed() {
        robot.getBottomFlywheel().setRPS(robot.getBottomFlywheel().getConfiguredRPS() - 1);
    }
}