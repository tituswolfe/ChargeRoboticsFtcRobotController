package org.firstinspires.ftc.teamcode.robots.base.opmodes;



import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.pedropathing.util.Timer;

import org.firstinspires.ftc.teamcode.robots.base.GamepadMapping;
import org.firstinspires.ftc.teamcode.robots.base.RobotBase;
import org.firstinspires.ftc.teamcode.robots.base.StaticData;

/**
 * {@link OpModeBase} is an abstract subclass of {@link OpMode}.
 * Use {@link TeleOpBase} or {@link BaseAuto} to create your {@link OpMode}!
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


    protected Timer opmodeTimer = new Timer();
    private final static Timer sleepTimer = new Timer();
    protected boolean isEndgame = false;

    // R.I.P AlliancePosition, OpModeType, & FieldType

    /**
     * Initiates and instantiates hardware & handlers.
     */
    @Override
    public void init() {
        telemetry.addLine("Charger Robotics");
        telemetry.addLine("Please wait . . .");
        telemetry.update();

        robot = instantiateRobot();
        gamepadMapping1 = instantiateGamepadMapping1();
        gamepadMapping2 = instantiateGamepadMapping2();
        robot.init(hardwareMap, instantiateStartPose(), instantiateAllianceColor());

        if (robot.getFollower() != null) buildPaths(robot.getFollower());

        opModeTypeSpecificInit();

        telemetry.addLine("READY");
        telemetry.update();
    }

    @Override
    public void init_loop() {
        // TODO: Auto select options
        super.init_loop();
    }

    public abstract void opModeTypeSpecificInit();
    public abstract void buildPaths(Follower follower);


    @Override
    public void start() {
        opmodeTimer.resetTimer();
    }

    @Override
    public void loop() {
        robot.updateHardwareStates();

        updateTelemetry(telemetryManager);
    }

    public void updateTelemetry(TelemetryManager telemetry) {
        telemetry.addLine("");


        if (robot.getFollower() != null) {
            telemetry.addLine("- DRIVETRAIN -");
            telemetry.addData("x", robot.getFollower().getPose().getX());
            telemetry.addData("y", robot.getFollower().getPose().getY());
            telemetry.addData("heading", Math.toDegrees(robot.getFollower().getPose().getHeading()));
            telemetry.addData("Field Centric Offset (Deg)", Math.toDegrees(robot.fieldCentricOffset));
            telemetry.addData("isSlowMode", robot.isSlowMode);
            telemetry.addLine("");
        }

        telemetry.addLine("- OpMode info -");
        telemetry.addData("Alliance Color", StaticData.allianceColor);
        telemetry.addData("Elapsed time (sec)", opmodeTimer.getElapsedTimeSeconds());
        telemetry.addData("isEndgame", isEndgame);
        telemetry.addLine("");
        telemetry.addLine("- Charger Robotics -");
        telemetry.addLine("- DON'T TOUCH THAT RYAN! -");

        telemetryManager.update(this.telemetry);
    }

    public boolean isEndgame() {
        return isEndgame;
    }

    protected abstract Robot instantiateRobot();
    protected abstract Pose instantiateStartPose();
    protected abstract GamepadMapping<Robot> instantiateGamepadMapping1();
    protected abstract GamepadMapping<Robot> instantiateGamepadMapping2();
    protected abstract AllianceColor instantiateAllianceColor();
}
