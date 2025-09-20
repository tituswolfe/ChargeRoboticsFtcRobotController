package org.firstinspires.ftc.teamcode.robots.decode;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.robots.base.GamepadControllerBase;

public class NealsonGamepadController extends GamepadControllerBase<DecodeRobot> {

    public NealsonGamepadController(DecodeRobot robot, Gamepad gamepad1, Gamepad gamepad2) {
        super(robot, gamepad1, gamepad2);
    }

    @Override
    protected void proceessGamepad1(Gamepad gamepad) {

    }

    @Override
    protected void proceessGamepad2(Gamepad gamepad) {

    }
}
