package org.firstinspires.ftc.teamcode.robots.base;

import com.qualcomm.robotcore.hardware.Gamepad;

public abstract class GamepadControllerBase<Robot extends RobotBase> {
    protected final Robot robot;
    protected final Gamepad gamepad1;
    protected final Gamepad gamepad2;

    private long gamepad1UnlockTime = 0; // System time (in mills) when controls can unlock
    private long gamepad2UnlockTime = 0;


    public GamepadControllerBase(Robot opMode, Gamepad gamepad1, Gamepad gamepad2) {
        this.robot = opMode;
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
    }

    public void processGamepadControls() {
        if(gamepad1UnlockTime < System.currentTimeMillis()) {
            proceessGamepad1(gamepad1);
        }
        if (gamepad2UnlockTime < System.currentTimeMillis()) {
            proceessGamepad2(gamepad2);
        }
    }

    protected abstract void proceessGamepad1(Gamepad gamepad);
    protected abstract void proceessGamepad2(Gamepad gamepad);

    public void lockOutGamepad1() {
        lockOutGamepad1(200);
    }

    public void lockOutGamepad1(int durationMills) {
        gamepad1UnlockTime = System.currentTimeMillis() + durationMills;
    }

    public void lockOutGamepad2() {
        lockOutGamepad2(200);
    }

    public void lockOutGamepad2(int durationMills) {
        gamepad2UnlockTime = System.currentTimeMillis() + durationMills;
    }
}
