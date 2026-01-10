package org.firstinspires.ftc.teamcode.robots.season.decode.jetfire;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.robots.base.GamepadMapping;

public class JetFireGamepadMapping2 extends GamepadMapping<JetfireRobot> {
    public JetFireGamepadMapping2(JetfireRobot jetfireRobot) {
        super(jetfireRobot);
    }

    @Override
    public void onYPressed() {
        Pose limelightPose = robot.getLimelightHandler().getPose();
        Pose currentPose = robot.getFollower().getPose(); // TODO: Fix
        if (limelightPose != null) {
            robot.getFollower().setPose(new Pose(limelightPose.getX(), limelightPose.getY(), currentPose.getHeading()));
        }
    }

    @Override
    public void onBPressed() {

    }

    @Override
    public void onAPressed() {

    }

    @Override
    public void onXPressed() {

    }

    @Override
    public void leftJoystick(float x, float y) {

    }

    @Override
    public void rightJoystick(float x, float y) {

    }

    @Override
    public void leftTrigger(float val) {

    }

    @Override
    public void rightTrigger(float val) {

    }

    @Override
    public void onLeftTriggerPressed() {

    }

    @Override
    public void onRightTriggerPressed() {

    }

    @Override
    public void onLeftTriggerReleased() {

    }

    @Override
    public void onRightTriggerReleased() {

    }

    @Override
    public void onLeftBumperPressed() {

    }

    @Override
    public void onRightBumperPressed() {

    }

    @Override
    public void onDpadUpPressed() {

    }

    @Override
    public void onDpadRightPressed() {
        if (robot.isInFarZone()) {
            JetfireRobot.FAR_ZONE_TURNTABLE_OFFSET_DEG--;
        } else {
            JetfireRobot.CLOSE_ZONE_TURNTABLE_OFFSET_DEG--;
        }
    }

    @Override
    public void onDpadDownPressed() {

    }

    @Override
    public void onDpadLeftPressed() {
        if (robot.isInFarZone()) {
            JetfireRobot.FAR_ZONE_TURNTABLE_OFFSET_DEG++;
        } else {
            JetfireRobot.CLOSE_ZONE_TURNTABLE_OFFSET_DEG++;
        }
    }

    @Override
    public void joysticks(float leftX, float leftY, float rightX, float rightY) {
        // super.joysticks(leftX, leftY, rightX, rightY);
    }

    @Override
    public void onRightStickPressed() {
        //super.onRightStickPressed();
    }
}
