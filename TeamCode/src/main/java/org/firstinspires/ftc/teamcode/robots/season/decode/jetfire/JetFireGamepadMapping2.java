package org.firstinspires.ftc.teamcode.robots.season.decode.jetfire;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.hardware.controllers.servo.RGBIndicatorLightController;
import org.firstinspires.ftc.teamcode.robots.base.GamepadMapping;
import org.firstinspires.ftc.teamcode.robots.base.StaticData;
import org.firstinspires.ftc.teamcode.robots.base.opmodes.OpModeBase;

public class JetFireGamepadMapping2 extends GamepadMapping<JetfireRobot> {
    public JetFireGamepadMapping2(JetfireRobot jetfireRobot) {
        super(jetfireRobot);
    }

    @Override
    public void onYPressed() {
        robot.getFollower().setHeading(StaticData.allianceColor == OpModeBase.AllianceColor.RED ? 90 : -90);
    }

    @Override
    public void onBPressed() {
        robot.getFollower().setHeading(StaticData.allianceColor == OpModeBase.AllianceColor.RED ? 0 : 180);
//        if (JetfireRobot.TUNING) {
//            robot.setChamberKalmanFilter(new KalmanFilter(JetfireRobot.chamberFilterParameters));
//        }
    }

    @Override
    public void onAPressed() {
        robot.getFollower().setHeading(StaticData.allianceColor == OpModeBase.AllianceColor.RED ? -90 : 90);

    }

    @Override
    public void onXPressed() {
        robot.getFollower().setHeading(StaticData.allianceColor == OpModeBase.AllianceColor.RED ? 180 : 0);

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
        Pose limelightPose = robot.getLimelightHandler().getPose();
        Pose currentPose = robot.getFollower().getPose();

        if (limelightPose != null) {
            robot.getFollower().setPose(new Pose(limelightPose.getX(), limelightPose.getY(), currentPose.getHeading()));
            robot.indicate(RGBIndicatorLightController.Color.VIOLET);
        }
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
