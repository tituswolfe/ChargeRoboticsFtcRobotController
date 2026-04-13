package org.firstinspires.ftc.teamcode.robots.base;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.util.math.Angle;

public abstract class PrimaryDriverGamepadMapping<Robot extends RobotBase> extends GamepadMapping<Robot> {
    public PrimaryDriverGamepadMapping(Robot robot, Gamepad gamepad) {
        super(robot, gamepad);
    }

    @Override
    public void joysticks(float leftX, float leftY, float rightX, float rightY) {
        if (robot.getFollower() == null) {
            return;
        }

        if (!robot.getFollower().isTeleopDrive()) {
            return;
        }


        //robot.getFollower().setTeleOpDrive(smooth(leftY), smooth(leftX), smooth(rightX), robot.isRobotCentric, (robot.isRobotCentric ? 0 : robot.fieldCentricOffset.getAngle(Angle.AngleNormalization.NONE)));

        robot.getFollower().setTeleOpDrive(leftY * robot.speedFactor, leftX * robot.speedFactor, rightX * robot.speedFactor, robot.isRobotCentric, (robot.isRobotCentric ? 0 : robot.fieldCentricOffset.getAngle(Angle.AngleNormalization.NONE)));
    };

    // cubic?
    public static double smooth(double input) {
        return (0.7 * Math.pow(input, 3)) + (0.3 * input);
    }

    @Override
    public void onLeftStickPressed() {
        robot.isRobotCentric = !robot.isRobotCentric;
    }

    @Override
    public void onRightStickPressed() {
        Robot.isSlowMode = !Robot.isSlowMode;
        if (Robot.isSlowMode) {
            robot.speedFactor = RobotBase.slowSpeedFactor;
        } else {
            robot.speedFactor = 1;
        }
    }
}
