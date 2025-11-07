package org.firstinspires.ftc.teamcode.robots.jetfire.opmodes.teleops;

import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.controllers.servo.RGBIndicatorLightController;
import org.firstinspires.ftc.teamcode.robots.base.GamepadMapping;
import org.firstinspires.ftc.teamcode.robots.base.opmodes.TeleOpBase;
import org.firstinspires.ftc.teamcode.robots.jetfire.JetFireRobot;
import org.firstinspires.ftc.teamcode.robots.jetfire.JetFireGamepadMapping;
import org.firstinspires.ftc.teamcode.robots.jetfire.JetFireGamepadMapping2;


@TeleOp(name = "JetFire")
public class Teleop extends TeleOpBase<JetFireRobot> {
    boolean isRed;
    @Override
    public void loop() {
        double topFlywheelCurrentRPS = robot.getTopFlywheel().getCurrentRPS();
        double bottomFlywheelCurrentRPS = robot.getBottomFlywheel().getCurrentRPS();

        if (isWithinRange(topFlywheelCurrentRPS, robot.getFlywheelSpeedFactor() * JetFireRobot.topFlywheelBaseSpeed,2) && isWithinRange(bottomFlywheelCurrentRPS, robot.getFlywheelSpeedFactor() * JetFireRobot.bottomFlywheelBaseSpeed, 2)) {
            switch (robot.flywheelDistancePreset) {
                case OFF:
                    robot.getRgbIndicatorLightController().setColor(RGBIndicatorLightController.Color.OFF);
                    break;
                case CLOSE:
                    robot.getRgbIndicatorLightController().setColor(RGBIndicatorLightController.Color.GREEN);
                    break;
                case FAR:
                    robot.getRgbIndicatorLightController().setColor(RGBIndicatorLightController.Color.VIOLET);
                    break;
            }
        } else {
            robot.getRgbIndicatorLightController().setColor(RGBIndicatorLightController.Color.RED);
        }

        super.loop();
    }

    @Override
    public void updateTelemetry(TelemetryManager telemetry) {
        telemetry.addLine("- FLYWHEEL DATA -");
        telemetry.addData("Top Flywheel RPS", robot.getTopFlywheel().getCurrentRPS());
        telemetry.addData("Bottom Flywheel RPS", robot.getBottomFlywheel().getCurrentRPS());
        telemetry.addLine("");
        telemetry.addData("top target RPS", robot.getTopFlywheel().getConfiguredRPS());
        telemetry.addData("bottom target RPS", robot.getBottomFlywheel().getConfiguredRPS());
        telemetry.addLine("");
        telemetry.addData("top pow", robot.getTopFlywheel().getDcMotorEx().getPower());
        telemetry.addData("bottom pow", robot.getBottomFlywheel().getDcMotorEx().getPower());


        super.updateTelemetry(telemetry);
    }

    @Override
    protected JetFireRobot instantiateRobot() {
        return new JetFireRobot();
    }

    @Override
    protected GamepadMapping instantiateGamepadMapping1() {
        return new JetFireGamepadMapping(robot);
    }

    @Override
    protected GamepadMapping instantiateGamepadMapping2() {
        return null; // new JetFireGamepadMapping2(robot);
    }


    public static boolean isWithinRange(double val, double target, double margin) {
        return val > target - margin && val < target + margin;
    }
}
