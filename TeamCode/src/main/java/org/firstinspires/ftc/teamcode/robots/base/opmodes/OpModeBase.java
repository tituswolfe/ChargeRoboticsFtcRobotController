package org.firstinspires.ftc.teamcode.robots.base.opmodes;

import com.bylazar.field.FieldManager;
import com.bylazar.field.PanelsField;
import com.bylazar.field.Style;
import com.bylazar.gamepad.PanelsGamepad;
import com.bylazar.lights.PanelsLights;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.pedropathing.util.Timer;

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

    protected Timer opmodeTimer = new Timer();
    protected boolean isEndgame = false;

    private final Timer deltaTimer = new Timer();

    /**
     * Initiates and instantiates hardware & handlers.
     */
    @Override
    public void init() {
        // TODO: Panels field
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

        AllianceColor allianceColor = Optional.ofNullable(instantiateAllianceColor())
                .orElse(OpModeBase.AllianceColor.BLUE);
        StaticData.allianceColor = allianceColor;

        robot.init(hardwareMap, startPose, allianceColor);

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

        //drawRobot(robot.getFollower().getPose(), robotLook);

        telemetryManager.update(this.telemetry);
    }

    /**
     * This draws a robot at a specified Pose with a specified
     * look. The heading is represented as a line.
     *
     * @param pose  the Pose to draw the robot at
     * @param style the parameters used to draw the robot with
     */
    public static void drawRobot(Pose pose, Style style) {
        if (pose == null || Double.isNaN(pose.getX()) || Double.isNaN(pose.getY()) || Double.isNaN(pose.getHeading())) {
            return;
        }

        panelsField.setStyle(style);
        panelsField.moveCursor(pose.getX(), pose.getY());
        panelsField.circle(9);

        Vector v = pose.getHeadingAsUnitVector();
        v.setMagnitude(v.getMagnitude() * 9);
        double x1 = pose.getX() + v.getXComponent() / 2, y1 = pose.getY() + v.getYComponent() / 2;
        double x2 = pose.getX() + v.getXComponent(), y2 = pose.getY() + v.getYComponent();

        panelsField.setStyle(style);
        panelsField.moveCursor(x1, y1);
        panelsField.line(x2, y2);
    }

    public boolean isEndgame() {
        return isEndgame;
    }

    protected abstract Robot instantiateRobot();
    protected abstract Pose instantiateStartPose();
    protected abstract AllianceColor instantiateAllianceColor();
    protected abstract GamepadMapping<Robot> instantiateGamepadMapping1();
    protected abstract GamepadMapping<Robot> instantiateGamepadMapping2();
}
