package org.firstinspires.ftc.teamcode.robots.base;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.ThreadUtil;

import dev.frozenmilk.dairy.pasteurized.Pasteurized;

public abstract class BaseOpMode<Robot extends RobotBase, GamepadController extends GamepadControllerBase> extends OpMode {
    protected Robot robot;
    protected GamepadController gamepadController;

    private boolean isOpModeSetup = false;

    public void setupOpMode(Robot robot, GamepadController gamepadController) {
        this.robot = robot;
        this.gamepadController = gamepadController;
        isOpModeSetup = true;
    }

    /**
     * Override this method, and call setupOpMode(), before calling super().
     */
    @Override
    public void init() {
        Pasteurized.gamepad1();
        if(!isOpModeSetup) {
            telemetry.addLine("OpMode was not setup. Please call setupOpMode() in init before super().");
            telemetry.update();
            stop();
        }

        telemetry.addLine("Charger Robotics");
        telemetry.addLine("Please wait . . .");
        telemetry.update();
        robot.init(hardwareMap);
        telemetry.addLine("READY");
    }

    @Override
    public void loop() {
        telemetry.addLine("- Charger Robotics -");
        telemetry.addLine("- DON'T TOUCH THAT RYAN! -");
        telemetry.update();
    }

    @Override
    public void start() {

    }

    @Override
    public void stop() {
        ThreadUtil.shutdown();
    }
}
