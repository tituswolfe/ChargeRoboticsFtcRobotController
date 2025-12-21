package org.firstinspires.ftc.teamcode.robots.base.opmodes;



import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.pedropathing.util.Timer;

import org.firstinspires.ftc.teamcode.robots.base.GamepadMapping;
import org.firstinspires.ftc.teamcode.robots.base.RobotBase;

import java.util.Optional;

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

    private long deltaTime = 0;
    private Timer deltaTimer = new Timer();
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

//        if (allianceColor != null) {
//            StaticData.allianceColor = allianceColor;
//        } else if (StaticData.allianceColor == null) {
//            StaticData.allianceColor = OpModeBase.AllianceColor.BLUE; // DEFAULT TO BLUE
//        }

        Pose startPose = Optional.ofNullable(instantiateStartPose())
                .orElse(new Pose(0, 0));

        AllianceColor allianceColor = Optional.ofNullable(instantiateAllianceColor())
                .orElse(OpModeBase.AllianceColor.BLUE);

        robot.init(hardwareMap, startPose, allianceColor);

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
        deltaTime = deltaTimer.getElapsedTime();
        deltaTimer.resetTimer();

        robot.update(deltaTime);
        updateTelemetry(telemetryManager);
    }

    public void updateTelemetry(TelemetryManager telemetry) {

        telemetry.addLine("- OpMode info -");
        // telemetry.addData("Alliance Color", StaticData.allianceColor);
        telemetry.addData("Elapsed time (sec)", opmodeTimer.getElapsedTimeSeconds());
        telemetry.addData("isEndgame", isEndgame);
        telemetry.addData("Delta Time", deltaTime);


        robot.hardwareTelemetry(telemetry);

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
    protected abstract AllianceColor instantiateAllianceColor();
    protected abstract GamepadMapping<Robot> instantiateGamepadMapping1();
    protected abstract GamepadMapping<Robot> instantiateGamepadMapping2();
}
