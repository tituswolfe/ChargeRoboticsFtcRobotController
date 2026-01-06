package org.firstinspires.ftc.teamcode.robots.base.opmodes;

import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.util.Timer;

import org.firstinspires.ftc.teamcode.robots.base.GamepadMapping;
import org.firstinspires.ftc.teamcode.robots.base.RobotBase;
import org.firstinspires.ftc.teamcode.robots.base.StaticData;

public abstract class BaseAuto<Robot extends RobotBase> extends OpModeBase<Robot> {
    protected Timer pathTimer = new Timer();
    protected Timer actionTimer = new Timer();
    private int pathState = 0;

    /**
     * See <a href="https://pedropathing.com/docs/pathing/examples/auto">Example Auto (Pedro Pathing)</a>
     * This method is continously called in {@link #loop()}. Use a state machine (switch case) to manage which path is active.
     *
     * @param pathState current path state/segment
     */
    public abstract void autonomousPathUpdate(int pathState);

    @Override
    public void opModeTypeSpecificInit() {
        robot.startConfiguration();
    }

//    @Override
//    public void updateTelemetry(TelemetryManager telemetry) {
//        telemetry.addData("Path State", pathState);
//        super.updateTelemetry(telemetry);
//    }

    @Override
    public void init() {
        StaticData.previouslyInAuto = true;
        super.init();
    }

    @Override
    public void start() {
        pathTimer.resetTimer();
        actionTimer.resetTimer();
        super.start();
    }

    @Override
    public void loop() {
        autonomousPathUpdate(pathState);
        super.loop();
    }

    public void setPathState(int pState, boolean resetActionTimer) {
        pathState = pState;
        pathTimer.resetTimer();
        if (resetActionTimer) actionTimer.resetTimer();
    }

    @Override
    protected GamepadMapping<Robot> instantiateGamepadMapping1() {
        return null;
    }

    @Override
    protected GamepadMapping<Robot> instantiateGamepadMapping2() {
        return null;
    }
}
