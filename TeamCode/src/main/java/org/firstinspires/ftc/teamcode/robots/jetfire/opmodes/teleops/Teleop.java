package org.firstinspires.ftc.teamcode.robots.jetfire.opmodes.teleops;


import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.field.PanelsField;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.teamcode.robots.base.GamepadMapping;
import org.firstinspires.ftc.teamcode.robots.base.opmodes.TeleOpBase;
import org.firstinspires.ftc.teamcode.robots.jetfire.JetFireRobot;
import org.firstinspires.ftc.teamcode.robots.jetfire.JetFireGamepadMapping;

import java.time.temporal.ValueRange;

@TeleOp(name = "JetFire")
public class Teleop extends TeleOpBase<JetFireRobot> {
    boolean isRed;
    @Override
    public void loop() {
        double topFlywheelCurrentRPS = robot.getTopFlywheel().getCurrentRPS();
        double bottomFlywheelCurrentRPS = robot.getBottomFlywheel().getCurrentRPS();

//        double topFlywheelConfiguredRPS = robot.getTopFlywheel().getConfiguredRPS();
//        double bottomFlywheelConfiguredRPS = robot.getBottomFlywheel().getConfiguredRPS();

        if (isWithinRange(topFlywheelCurrentRPS, robot.getFlywheelSpeedFactor() * JetFireRobot.topFlywheelBaseSpeed,2) && isWithinRange(bottomFlywheelCurrentRPS, robot.getFlywheelSpeedFactor() * JetFireRobot.bottomFlywheelBaseSpeed, 2)) {
            switch (robot.flywheelDistancePreset) {
                case OFF:
                    if (isRed) {
                        robot.indicatorRed();
                        isRed = true;
                    }
                    isRed = false;
                    break;
                case CLOSE:
                    robot.indicatorGreen();
                    break;
                case FAR:
                    robot.indicatorIndigo();
                    break;
            }
        }
//
//        if (topFlywheelCurrentRPS > topFlywheelConfiguredRPS - 2 && topFlywheelCurrentRPS < topFlywheelConfiguredRPS + 2 && bottomFlywheelCurrentRPS > bottomFlywheelConfiguredRPS - 2 && bottomFlywheelCurrentRPS < bottomFlywheelConfiguredRPS + 2) {
//        } else {
//            robot.indicatorRed();
//        }

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
        return null;
    }


    public boolean isWithinRange(double val, double target, double margin) {
        return val > target - margin && val < target + margin;
    }
}
