package org.firstinspires.ftc.teamcode.robots.base.opmodes;

import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;

import org.firstinspires.ftc.teamcode.robots.base.GamepadMapping;
import org.firstinspires.ftc.teamcode.robots.base.RobotBase;
import org.firstinspires.ftc.teamcode.robots.base.StaticData;

import java.util.ArrayList;

public abstract class BaseAuto<Robot extends RobotBase> extends OpModeBase<Robot> {
    protected Timer pathTimer = new Timer();
    protected Timer actionTimer = new Timer();
    private int pathState = 0;

    public static double END_OF_PATH_T_VALUE = 0.97;

    protected ArrayList<Pose> pathPoses;

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

    @Override
    public void init() {
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

        telemetryManager.addLine("- AUTO -");
        telemetryManager.addData("Path State", pathState);
        telemetryManager.addLine("");

        super.loop();
    }

    public abstract ArrayList<Pose> generatePoses();

//    public void nextPath(PathChain path, int pState) {
//        if (isAtEndOfPathChain()) {
//            robot.getFollower().followPath(path);
//            setPathState(pState, true);
//        }
//    }
//
//    public boolean isAtEndOfPathChain() {
//        return robot.getFollower().getCurrentTValue() > END_OF_PATH_T_VALUE && robot.getFollower().getCurrentPathNumber() + 1 >= robot.getFollower().getCurrentPathChain().size();
//    }

    public void setPathState(int pState, boolean resetActionTimer) {
        pathState = pState;
        pathTimer.resetTimer();
        if (resetActionTimer) actionTimer.resetTimer();
    }

    public void setPathState(int pState) {
        setPathState(pState, true);
    }

    public void nextPathState() {
        setPathState(pathState + 1, true);
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
