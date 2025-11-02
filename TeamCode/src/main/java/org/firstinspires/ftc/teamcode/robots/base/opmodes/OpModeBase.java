package org.firstinspires.ftc.teamcode.robots.base.opmodes;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robots.base.GamepadHandlerBase;
import org.firstinspires.ftc.teamcode.robots.base.RobotBase;
import org.firstinspires.ftc.teamcode.robots.base.StaticData;

/**
 * {@link OpModeBase} is an abstract subclass of {@link OpMode}.
 * Use {@link TeleOpBase} or {@link BaseAuto} to create your {@link OpMode}!
 * {@link OpModeBase} takes generics for a subclasses of {@link RobotBase} and of {@link GamepadHandlerBase}.
 *
 * @author Titus Wolfe
 */
public abstract class OpModeBase<Robot extends RobotBase, GamepadHandler1 extends GamepadHandlerBase, GamepadHandler2 extends GamepadHandlerBase> extends OpMode {
    protected Robot robot;
    protected GamepadHandler1 gamepadHandler1;
    protected GamepadHandler2 gamepadHandler2;

    public enum AllianceColor {
        RED,
        BLUE
    }
    private AllianceColor allianceColor;

    public enum OpModeType {
        AUTO,
        TELEOP
    }
    private OpModeType opModeType;

    // R.I.P AlliancePosition (interesting, but unnecessary after further thought)

    private boolean isEndgame = false;
    private Timer opmodeTimer, actionTimer;

    private Timer sleepTimer = new Timer();
    boolean isActiveSleep = false;

    /**
     * Initiates and instantiates hardware & handlers.
     */
    @Override
    public void init() {
        telemetry.addLine("Charger Robotics");
        telemetry.addLine("Please wait . . .");
        telemetry.update();

        allianceColor = instantiateAllianceColor();
        StaticData.lastAllianceColor = allianceColor;
        opModeType = instantiateOpModeType();

        robot = instantiateRobot();
        gamepadHandler1 = instantiateGamepadHandler1();
        gamepadHandler2 = instantiateGamepadHandler2();
        robot.init(hardwareMap, instantiateStartPose());

        if (opModeType == OpModeType.AUTO) robot.startConfiguration();

        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        telemetry.addLine("READY");
        telemetry.update();
    }

    public void instantiateComponents() {

    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
    }

    @Override
    public void loop() {
        if (!isActiveSleep) {
            if (gamepadHandler1 != null) gamepadHandler1.processGamepadControls();
            if (gamepadHandler2 != null) gamepadHandler2.processGamepadControls();
        }

        if (opmodeTimer.getElapsedTimeSeconds() > 150) isEndgame = true;

        telemetry.addLine();

        if (robot.getFollower() != null) {
            robot.getFollower().update();

            telemetry.addLine("- Position -");
            telemetry.addData("x", robot.getFollower().getPose().getX());
            telemetry.addData("y", robot.getFollower().getPose().getY());
            telemetry.addData("heading", Math.toDegrees(robot.getFollower().getPose().getHeading()));
            telemetry.addData("isSlowMode", GamepadHandlerBase.isSlowMode);
            telemetry.addLine();
        }

        telemetry.addLine("- OpMode info -");
        telemetry.addData("Elapsed time (sec)", opmodeTimer.getElapsedTimeSeconds());
        telemetry.addData("isEndgame", isEndgame);
        telemetry.addLine();
        telemetry.addLine("- Charger Robotics -");
        telemetry.addLine("- DON'T TOUCH THAT RYAN! -");
        telemetry.update();
    }

    public boolean isEndgame() {
        return isEndgame;
    }


    public void activeSleep(long mills) {
        sleepTimer.resetTimer();
        isActiveSleep = true;
        while(sleepTimer.getElapsedTime() < mills) {
            loop();
        }
        isActiveSleep = false;
    }

    protected abstract Robot instantiateRobot();
    protected abstract Pose instantiateStartPose();
    protected abstract GamepadHandler1 instantiateGamepadHandler1();
    protected abstract GamepadHandler2 instantiateGamepadHandler2();
    protected abstract AllianceColor instantiateAllianceColor();
    protected abstract OpModeType instantiateOpModeType();

    public OpModeType getOpModeType() {
        return opModeType;
    }

    public AllianceColor getAllianceColor() {
        return allianceColor;
    }
}
