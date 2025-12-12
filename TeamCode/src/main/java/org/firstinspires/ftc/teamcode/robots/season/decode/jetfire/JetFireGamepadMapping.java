package org.firstinspires.ftc.teamcode.robots.season.decode.jetfire;

import org.firstinspires.ftc.teamcode.robots.base.GamepadMapping;
import org.firstinspires.ftc.teamcode.robots.base.StaticData;
import org.firstinspires.ftc.teamcode.robots.base.opmodes.OpModeBase;

public class JetFireGamepadMapping extends GamepadMapping<JetFireRobot> {
    boolean isIntakeActive = false;
    boolean headingLock = false;

    public JetFireGamepadMapping(JetFireRobot decodeRobot) {
        super(decodeRobot);
    }

    @Override
    public void onYPressed() {
        robot.pushArtifact();
    }

    @Override
    public void onBPressed() {
        robot.setAllianceColor((StaticData.allianceColor == OpModeBase.AllianceColor.BLUE) ? OpModeBase.AllianceColor.RED : OpModeBase.AllianceColor.BLUE);
    }

    @Override
    public void onAPressed() {
    }

    @Override
    public void onXPressed() {
        headingLock = !headingLock;
    }

    @Override
    public void leftJoystick(float x, float y) {

    }

    @Override
    public void rightJoystick(float x, float y) {

    }


    @Override
    public void joysticks(float leftX, float leftY, float rightX, float rightY) {
        if (robot.getFollower() == null) return;
        if (!robot.getFollower().isTeleopDrive()) return;

        robot.getFollower().setTeleOpDrive(
                leftY * robot.speedFactor,
                leftX * robot.speedFactor,
                (headingLock ? robot.getHeadingPIDFController().run() : rightX * robot.speedFactor),
                robot.isRobotCentric,
                (robot.isRobotCentric ? 0 : robot.fieldCentricOffset)
        );
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
            case AUTO -> JetFireRobot.FlywheelSpeedMode.OFF;
            case MANUEL -> JetFireRobot.FlywheelSpeedMode.OFF;
            case OFF -> JetFireRobot.FlywheelSpeedMode.AUTO;
        });
    }

    @Override
    public void onDpadUpPressed() {
        robot.flywheelSpeed += 100;
    }

    @Override
    public void onDpadRightPressed() {
        robot.headingGoalOffsetDeg -= 1;
    }

    @Override
    public void onDpadDownPressed() {
        robot.flywheelSpeed -= 100;

    }

    @Override
    public void onDpadLeftPressed() {
        robot.headingGoalOffsetDeg += 1;
    }
}