package org.firstinspires.ftc.teamcode.robots.base.opmodes;


import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.robots.base.GamepadHandlerBase;
import org.firstinspires.ftc.teamcode.robots.base.RobotBase;

/**
 * {@link BaseOpMode} is an abstract subclass of {@link OpMode}.
 * Extend {@link BaseOpMode} to create {@link  com.qualcomm.robotcore.eventloop.opmode.TeleOp} and {@link com.qualcomm.robotcore.eventloop.opmode.Autonomous}.
 * {@link BaseOpMode} takes generics for a subclasses of {@link RobotBase} and of {@link GamepadHandlerBase}.
 * @author Titus Wolfe
 */
public abstract class BaseOpMode<Robot extends RobotBase, GamepadHandler1 extends GamepadHandlerBase, GamepadHandler2 extends GamepadHandlerBase> extends OpMode {
    protected Robot robot;
    protected GamepadHandler1 gamepadHandler1;
    protected GamepadHandler2 gamepadHandler2;

    private boolean isEndgame = false;

    /**
     * Initiates and instantiates hardware & gamepad handlers.
     * Please wait to call {@link Robot#getFollower()} until {@link OpMode#start()}.
     */
    @Override
    public void init() {
        telemetry.addLine("Charger Robotics");
        telemetry.addLine("Please wait . . .");
        telemetry.update();

        robot = instantiateRobot();
        gamepadHandler1 = instantiateGamepadHandler1();
        gamepadHandler2 = instantiateGamepadHandler2();
        robot.init(hardwareMap);

        Pose startPose = instantiateStartPose();
        if (robot.getFollower() != null && startPose != null) {
            robot.getFollower().setPose(startPose);
        }

        telemetry.addLine("READY");
        telemetry.update();
    }

    @Override
    public void loop() {
        gamepadHandler1.processGamepadControls();
        gamepadHandler2.processGamepadControls();

        if (getRuntime() > 150) {
            isEndgame = true;
        }

        telemetry.addLine();
        telemetry.addLine("- Charger Robotics -");
        telemetry.addLine("- DON'T TOUCH THAT RYAN! -");
        telemetry.update();
    }

    public boolean isEndgame() {
        return isEndgame;
    }

    protected abstract Robot instantiateRobot();
    /**
     * Subclass defined method. Return null to use odometry reading. Return new {@link Pose} to set robot position.
     */
    protected abstract Pose instantiateStartPose();
    protected abstract GamepadHandler1 instantiateGamepadHandler1();
    protected abstract GamepadHandler2 instantiateGamepadHandler2();
}
