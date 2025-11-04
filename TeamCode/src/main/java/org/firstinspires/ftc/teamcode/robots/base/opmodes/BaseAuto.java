package org.firstinspires.ftc.teamcode.robots.base.opmodes;

import com.pedropathing.util.Timer;

import org.firstinspires.ftc.teamcode.robots.base.GamepadMapping;
import org.firstinspires.ftc.teamcode.robots.base.RobotBase;

public abstract class BaseAuto<Robot extends RobotBase> extends OpModeBase<Robot> {
    private Timer pathTimer, actionTimer;
    private int pathState;

    /**
     * See <a href="https://pedropathing.com/docs/pathing/examples/auto">Example Auto (Pedro Pathing)</a>
     * This method is continously called in {@link #loop()}. Use a state machine (switch case) to manage which path is active.
     *
     * @param pathState
     * @returns a new path state
     */
    public abstract void autonomousPathUpdate(int pathState);

    @Override
    public void opModeTypeSpecificInit() {
        robot.startConfiguration();
    }

    @Override
    public void start() {
        pathTimer = new Timer(); // TODO: Move to init
        setPathState(0);
        super.start();
    }

    @Override
    public void loop() {
        autonomousPathUpdate(pathState);
        super.loop();
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    @Override
    protected OpModeType instantiateOpModeType() {
        return OpModeType.AUTO;
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
