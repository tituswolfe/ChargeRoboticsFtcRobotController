package org.firstinspires.ftc.teamcode.robots.base.opmodes;

import static org.firstinspires.ftc.teamcode.util.ThreadUtil.sleep;

import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.robots.base.GamepadHandlerBase;
import org.firstinspires.ftc.teamcode.robots.base.RobotBase;

/**
 * {@link BaseOpMode} is an abstract subclass of {@link OpMode}.
 * Extend {@link BaseOpMode} to create {@link  com.qualcomm.robotcore.eventloop.opmode.TeleOp} and {@link com.qualcomm.robotcore.eventloop.opmode.Autonomous}.
 * {@link BaseOpMode} takes generics for a subclasses of {@link RobotBase} and of {@link GamepadHandlerBase}.
 *
 * @author Titus Wolfe
 */
public abstract class BaseOpMode<Robot extends RobotBase, GamepadHandler1 extends GamepadHandlerBase, GamepadHandler2 extends GamepadHandlerBase> extends OpMode {
    protected Robot robot;
    protected GamepadHandler1 gamepadHandler1;
    protected GamepadHandler2 gamepadHandler2; // TODO: Test instatiate here
    private TelemetryManager telemetryManager;

    public enum FieldType {
        DIAMOND,
        SQUARE,
        SQUARE_INVERTED_ALLIANCE
    }
    private FieldType fieldType;

    public enum AllianceColor {
        RED,
        BLUE
    }
    private AllianceColor allianceColor;

    public enum AlliancePosition {
        LEFT,
        RIGHT
    }
    private AlliancePosition alliancePosition;

    private boolean isEndgame = false;

    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;
// TODO: Panels telemetry


    /**
     * Initiates and instantiates hardware & handlers.
     * Please wait to call {@link Robot#getFollower()} until {@link OpMode#start()}.
     */
    @Override
    public void init() {
        telemetry.addLine("Charger Robotics");
        telemetry.addLine("Please wait . . .");
        telemetry.update();

        alliancePosition = instantiateAlliancePosition();
        fieldType = instantiateFieldType();

        robot = instantiateRobot();
        gamepadHandler1 = instantiateGamepadHandler1();
        gamepadHandler2 = instantiateGamepadHandler2();

        robot.init(hardwareMap);

        if (robot.getFollower() != null) {
            Pose startPose = instantiateStartPose();
            robot.getFollower().setStartingPose(startPose != null ? startPose : new Pose(0, 0, 0));
            robot.getFollower().update();
        }

        buildPaths();
        adjustHardwareBeforeStart();

        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        telemetry.addLine("READY");
        telemetry.update();
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
    }

    /**
     * Adjust any hardware to set or hold a starting configuration for {@link com.qualcomm.robotcore.eventloop.opmode.Autonomous} before {@link #start()}.
     * You cannot have any CONTINOUS movement during init.
     * This method is called during init after initiating hardware and gamepad handlers.
     * The robot can use powered movement during {@link #init()} in {@link com.qualcomm.robotcore.eventloop.opmode.Autonomous} to set or hold a starting configuration.
     * Hardware cannot use POWERED movement during {@link #init()} in {@link com.qualcomm.robotcore.eventloop.opmode.TeleOp}.
     */
    protected abstract void adjustHardwareBeforeStart();

    protected abstract void buildPaths();

    @Override
    public void loop() {
        gamepadHandler1.processGamepadControls();
        gamepadHandler2.processGamepadControls();

        if ((opmodeTimer.getElapsedTime() * 1000) > 150) {
            isEndgame = true;
        }

        if (robot.getFollower() != null) {
            robot.getFollower().update();
            autonomousPathUpdate(pathState);

            telemetry.addLine();
            telemetry.addData("x", robot.getFollower().getPose().getX());
            telemetry.addData("y", robot.getFollower().getPose().getX());
            telemetry.addData("heading", Math.toDegrees(robot.getFollower().getPose().getHeading()));
        }

        telemetry.addLine("");
        telemetry.addLine("- Charger Robotics -");
        telemetry.addLine("- DON'T TOUCH THAT RYAN! -");
        telemetry.update();
    }
    protected abstract void autonomousPathUpdate(int pathState);

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
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
    protected abstract GamepadHandler2 instantiateGamepadHandler2();;
    protected abstract AlliancePosition instantiateAlliancePosition();;
    protected abstract FieldType instantiateFieldType();


    public FieldType getFieldType() {
        return fieldType;
    }

    public AllianceColor getAllianceColor() {
        return allianceColor;
    }

    public AlliancePosition getAlliancePosition() {
        return alliancePosition;
    }
}
