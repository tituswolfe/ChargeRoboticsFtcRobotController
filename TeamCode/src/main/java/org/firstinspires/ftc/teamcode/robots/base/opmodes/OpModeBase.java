package org.firstinspires.ftc.teamcode.robots.base.opmodes;


import com.bylazar.field.FieldManager;
import com.bylazar.field.PanelsField;
import com.bylazar.field.Style;
import com.bylazar.gamepad.PanelsGamepad;
import com.bylazar.lights.PanelsLights;
import com.bylazar.panels.Panels;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.pedropathing.util.Timer;

import org.firstinspires.ftc.teamcode.hardware.drivetrain.pedroPathing.Drawing;
import org.firstinspires.ftc.teamcode.robots.base.GamepadMapping;
import org.firstinspires.ftc.teamcode.robots.base.RobotBase;
import org.firstinspires.ftc.teamcode.robots.base.StaticData;
import org.firstinspires.ftc.teamcode.util.math.RollingAverage;

import java.util.Optional;
import java.util.concurrent.TimeUnit;

/**
 * {@link OpModeBase} is an abstract subclass of the SDK's {@link OpMode}.
 * Use {@link TeleOpBase} or {@link BaseAuto} to create your own {@link OpMode}!
 *
 * @author Titus Wolfe
 * @param <Robot> a {@link RobotBase} subclass
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

    protected static Timer opmodeTimer = new Timer();
    protected boolean isEndgame = false;

    private static final int DELTA_TIME_SAMPLE_SIZE = 50;
    RollingAverage rollingAverage = new RollingAverage(DELTA_TIME_SAMPLE_SIZE);

    long lastNanoTime;

    /**
     * Initiates and instantiates hardware & handlers.
     */
    @Override
    public void init() {
        // TODO: Make sure gamepads driing don't interfer with each other
        // TODO: Single static instance of robot and opmode

        telemetryManager.addLine("INITIALIZING: Charger Robotics 9808");
        updateTelemetry();

        robot = instantiateRobot();
        gamepadMapping1 = instantiateGamepadMapping1();
        gamepadMapping2 = instantiateGamepadMapping2();

        Pose startPose = Optional.ofNullable(instantiateStartPose())
                .orElse(new Pose(72, 72, Math.toRadians(270)));
        StaticData.lastPose = startPose;

        AllianceColor allianceColor = Optional.ofNullable(instantiateAllianceColor())
                .orElse(OpModeBase.AllianceColor.BLUE);
        StaticData.allianceColor = allianceColor;

        robot.init(hardwareMap, startPose, allianceColor);

        Drawing.init();

        if (robot.getFollower() != null) {
            buildPaths(robot.getFollower());
        }

        opModeTypeSpecificInit();

        telemetryManager.addLine("READY - Press PLAY to start");
        updateTelemetry();
    }

    @Override
    public void init_loop() {
        //super.init_loop();
        // TODO: MENU SELECT
    }

    public abstract void opModeTypeSpecificInit();
    public abstract void buildPaths(Follower follower);

    @Override
    public void start() {
        opmodeTimer.resetTimer();
    }

    @Override
    public void loop() {
        long currentNanoTime = System.nanoTime();
        long deltaTimeNs = currentNanoTime - lastNanoTime;
        long deltaTimeMs = TimeUnit.NANOSECONDS.toMillis(deltaTimeNs);
        lastNanoTime = currentNanoTime;

        rollingAverage.update(deltaTimeMs);

        telemetryManager.addLine("- OpMode info -");
        telemetryManager.addData("Alliance", StaticData.allianceColor);
        telemetryManager.addData("Loop Time (MS)", deltaTimeMs);
        telemetryManager.addData("Avrg. Loop Time (MS)", rollingAverage.getAverage());

        telemetryManager.addData("Elapsed time (sec)", opmodeTimer.getElapsedTimeSeconds());
        telemetryManager.addData("isEndgame", isEndgame);

        robot.update(deltaTimeNs, telemetryManager);

        telemetryManager.addLine("");
        telemetryManager.addLine("- CHARGER ROBOTICS 9808 -");
        telemetryManager.addLine("- DON'T TOUCH THAT RYAN! -");
        updateTelemetry();
    }

    private void updateTelemetry() {
        telemetryManager.update(this.telemetry);
    }

    @Override
    public void stop() {
        robot.stop();
        super.stop();
    }

    public boolean isEndgame() {
        return isEndgame;
    }

    public static long getOpModeElapsedTime() {
        return opmodeTimer.getElapsedTime();
    }

    public static double getOpModeElapsedTimeSeconds() {
        return opmodeTimer.getElapsedTimeSeconds();
    }

    protected abstract Robot instantiateRobot();
    protected abstract Pose instantiateStartPose();
    protected abstract AllianceColor instantiateAllianceColor();
    protected abstract GamepadMapping<Robot> instantiateGamepadMapping1();
    protected abstract GamepadMapping<Robot> instantiateGamepadMapping2();
}
