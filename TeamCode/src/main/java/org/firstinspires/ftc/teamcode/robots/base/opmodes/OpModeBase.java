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

import java.util.Optional;

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
    private static final FieldManager panelsField = PanelsField.INSTANCE.getField();
    private static final Style robotLook = new Style(
            "", "#3F51B5", 0.75
    );

    public enum AllianceColor {
        RED,
        BLUE
    }

    protected static Timer opmodeTimer = new Timer();
    protected boolean isEndgame = false;

    private final Timer deltaTimer = new Timer();

    /**
     * Initiates and instantiates hardware & handlers.
     */
    @Override
    public void init() {
        // TODO: Make sure gamepads driing don't interfer with each other
        // TODO: One static instance of robot and opmode etc.

        telemetry.addLine("Charger Robotics");
        telemetry.addLine("Please wait . . .");
        telemetry.update();

        robot = instantiateRobot();
        gamepadMapping1 = instantiateGamepadMapping1();
        gamepadMapping2 = instantiateGamepadMapping2();

        Pose startPose = Optional.ofNullable(instantiateStartPose())
                .orElse(new Pose(0, 0));
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

        telemetry.addLine("READY");
        telemetry.update();
    }

    @Override
    public void init_loop() {
        //super.init_loop();
    }

    public abstract void opModeTypeSpecificInit();
    public abstract void buildPaths(Follower follower);

    @Override
    public void start() {
        opmodeTimer.resetTimer();
    }

    @Override
    public void loop() {
        long deltaTimeMs = deltaTimer.getElapsedTime();
        deltaTimer.resetTimer();

        telemetryManager.addLine("- OpMode info -");
        telemetryManager.addData("Alliance Color", StaticData.allianceColor);
        telemetryManager.addData("Delta Time (MS)", deltaTimeMs);
        telemetryManager.addData("Elapsed time (sec)", opmodeTimer.getElapsedTimeSeconds());
        telemetryManager.addData("isEndgame", isEndgame);

        robot.update(deltaTimeMs, telemetryManager);

        telemetryManager.addLine("");
        telemetryManager.addLine("- Charger Robotics -");
        telemetryManager.addLine("- DON'T TOUCH THAT RYAN! -");


        panelsField.update();

        telemetryManager.update(this.telemetry);
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
