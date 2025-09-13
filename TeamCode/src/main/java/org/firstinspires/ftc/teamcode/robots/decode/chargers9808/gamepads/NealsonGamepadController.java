package org.firstinspires.ftc.teamcode.robots.decode.chargers9808.gamepads;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.robots.Decode.chargers9808.DecodeRobot9808;
import org.firstinspires.ftc.teamcode.robots.base.GamepadControllerBase;
import org.firstinspires.ftc.teamcode.robots.standard.StandardGamepadController;
import org.firstinspires.ftc.teamcode.robots.standard.StandardOpMode;

public class NealsonGamepadController extends GamepadControllerBase<DecodeRobot9808> {

    public NealsonGamepadController(DecodeRobot9808 robot, Gamepad gamepad1, Gamepad gamepad2) {
        super(robot, gamepad1, gamepad2);
    }

    @Override
    protected void proceessGamepad1(Gamepad gamepad) {

    }

    @Override
    protected void proceessGamepad2(Gamepad gamepad) {

    }
}
