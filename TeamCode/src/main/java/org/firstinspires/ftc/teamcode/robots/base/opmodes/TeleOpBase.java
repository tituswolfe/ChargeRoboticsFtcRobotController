package org.firstinspires.ftc.teamcode.robots.base.opmodes;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.robots.base.RobotBase;
import org.firstinspires.ftc.teamcode.robots.base.StaticData;

public abstract class TeleOpBase<Robot extends RobotBase> extends OpModeBase<Robot> {
    @Override
    public void start() {
        if (robot.getFollower() != null) robot.getFollower().startTeleopDrive();
        super.start();
    }

    @Override
    public void loop() {
        if (gamepadMapping1 != null) gamepadMapping1.processGamepad(gamepad1);
        if (gamepadMapping2 != null) gamepadMapping2.processGamepad(gamepad2);

        if (opmodeTimer.getElapsedTimeSeconds() > 150) isEndgame = true;

        if (robot.getFollower() != null && !robot.getFollower().isBusy() && !robot.getFollower().isTeleopDrive()) robot.getFollower().startTeleopDrive();

        super.loop();
    }

    @Override
    public void opModeTypeSpecificInit() {

    }

    @Override
    protected Pose instantiateStartPose() {
        return StaticData.lastPose;
    }

    @Override
    protected AllianceColor instantiateAllianceColor() {
        return StaticData.allianceColor;
    }
}
