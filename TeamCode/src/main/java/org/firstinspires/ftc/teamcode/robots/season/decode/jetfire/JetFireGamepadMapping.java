package org.firstinspires.ftc.teamcode.robots.season.decode.jetfire;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.robots.base.GamepadMapping;

public class JetFireGamepadMapping extends GamepadMapping<JetFireRobot> {
    boolean isIntakeActive = false;

    public JetFireGamepadMapping(JetFireRobot decodeRobot) {
        super(decodeRobot);
    }

    @Override
    public void onYPressed() {
        robot.pushArtifact();
    }

    @Override
    public void onBPressed() {

    }

    @Override
    public void onAPressed() {

    }

    @Override
    public void onXPressed() {
        Pose displacedPose = robot.getTargetGoal().minus(robot.getFollower().getPose());
        robot.getFollower().turnTo(Math.atan2(displacedPose.getY(), displacedPose.getX()));

//        Pose currentPos = robot.getFollower().getPose();
//        double targetAngle = Math.atan2(robot.targetGoal
//                .getY()-currentPos.getY(),robot.targetGoal
//                .getX()-currentPos.getX());
//        robot.getFollower().turnTo(targetAngle);
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
            robot.setIntakeMode(JetFireRobot.IntakeMode.REVERSE);
        } else if (isIntakeActive) {
            robot.setIntakeMode(JetFireRobot.IntakeMode.INTAKE);
        } else {
            robot.setIntakeMode(JetFireRobot.IntakeMode.OFF);
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
            robot.setIntakeMode(JetFireRobot.IntakeMode.INTAKE);
        } else {
            robot.setIntakeMode(JetFireRobot.IntakeMode.OFF);
        }
        // isIntakeActive = toggle(isIntakeActive, robot::setIntakeMode, robot::stopIntake);
    }

    @Override
    public void onRightBumperPressed() {
        robot.setFlywheelSpeedMode(switch (robot.getFlywheelSpeedMode()) {
            case AUTO -> JetFireRobot.FlywheelSpeedMode.MANUEL;
            case MANUEL -> JetFireRobot.FlywheelSpeedMode.OFF;
            case OFF -> JetFireRobot.FlywheelSpeedMode.AUTO;
        });

    }

    @Override
    public void onDpadUpPressed() {
        robot.flywheelSpeed++;
    }

    @Override
    public void onDpadRightPressed() {
    }

    @Override
    public void onDpadDownPressed() {

    }

    @Override
    public void onDpadLeftPressed() {
        robot.flywheelSpeed--;
    }
}