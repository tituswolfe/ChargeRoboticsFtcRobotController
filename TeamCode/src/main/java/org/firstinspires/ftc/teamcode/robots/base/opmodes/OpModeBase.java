package org.firstinspires.ftc.teamcode.robots.base.opmodes;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.pedropathing.util.Timer;

import org.firstinspires.ftc.teamcode.robots.base.GamepadMapping;
import org.firstinspires.ftc.teamcode.robots.base.RobotBase;
import org.firstinspires.ftc.teamcode.robots.base.StaticData;

/**
 * {@link OpModeBase} is an abstract subclass of {@link OpMode}.
 * Use {@link TeleOpBase} or {@link BaseAuto} to create your {@link OpMode}!
 * {@link OpModeBase} takes generics for a subclasses of {@link RobotBase} and of {@link GamepadMapping}.
 *
 * @author Titus Wolfe
 */
public abstract class OpModeBase<Robot extends RobotBase> extends OpMode {
    protected Robot robot;
    protected GamepadMapping<Robot> gamepadMapping1;
    protected GamepadMapping<Robot> gamepadMapping2;

    TelemetryManager telemetryManager = PanelsTelemetry.INSTANCE.getTelemetry();

    public enum AllianceColor {
        RED,
        BLUE
    }
    private AllianceColor allianceColor;

//    public enum OpModeType {
//        AUTO,
//        TELEOP
//    }
//    private OpModeType opModeType;

    // R.I.P AlliancePosition (interesting, but unnecessary after further thought)

    protected boolean isEndgame = false;
    protected Timer opmodeTimer = new Timer();
    private final static Timer sleepTimer = new Timer();

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
        gamepadMapping1 = instantiateGamepadMapping1();
        gamepadMapping2 = instantiateGamepadMapping2();
        robot.init(hardwareMap, instantiateStartPose(), allianceColor);

        opModeTypeSpecificInit();

        telemetry.addLine("READY");
        telemetry.update();
    }

    public abstract void opModeTypeSpecificInit();


    @Override
    public void start() {
        opmodeTimer.resetTimer();
    }

    @Override
    public void loop() {
        if (robot.getFollower() != null) {
            robot.getFollower().update();
            StaticData.lastPose = robot.getFollower().getPose();
        }

        updateTelemetry(telemetryManager);
    }

    public void updateTelemetry(TelemetryManager telemetry) {
        telemetry.addLine("");

        if (robot.getFollower() != null) {
            telemetry.addLine("- Position -");
            telemetry.addData("x", robot.getFollower().getPose().getX());
            telemetry.addData("y", robot.getFollower().getPose().getY());
            telemetry.addData("heading", Math.toDegrees(robot.getFollower().getPose().getHeading()));
            telemetry.addData("isSlowMode", robot.isSlowMode);
            telemetry.addLine("");
        }

        telemetry.addLine("- OpMode info -");
        telemetry.addData("Elapsed time (sec)", opmodeTimer.getElapsedTimeSeconds());
        telemetry.addData("isEndgame", isEndgame);
        telemetry.addLine("");
        telemetry.addLine("- Charger Robotics -");
        telemetry.addLine("- DON'T TOUCH THAT RYAN! -");
        telemetry.update();

        telemetryManager.update(this.telemetry);
    }

    public boolean isEndgame() {
        return isEndgame;
    }

    public static void activeSleep(long mills) {
        sleepTimer.resetTimer();
        while(sleepTimer.getElapsedTime() < mills);
    }

    protected abstract Robot instantiateRobot();
    protected abstract Pose instantiateStartPose();
    protected abstract GamepadMapping<Robot> instantiateGamepadMapping1();
    protected abstract GamepadMapping<Robot> instantiateGamepadMapping2();
    protected abstract AllianceColor instantiateAllianceColor();
    protected abstract OpModeType instantiateOpModeType();
}
