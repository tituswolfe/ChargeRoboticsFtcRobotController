package org.firstinspires.ftc.teamcode.robots.drivechassis;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.robots.base.GamepadHandlerBase;
import org.firstinspires.ftc.teamcode.robots.base.RobotBase;
import org.firstinspires.ftc.teamcode.robots.base.opmodes.BaseOpMode;

public class DriveGamepadHandler extends GamepadHandlerBase {
    public DriveGamepadHandler(RobotBase robotBase, BaseOpMode opMode, Gamepad gamepad, boolean allowDrive) {
        super(robotBase, opMode, gamepad, allowDrive);
    }

    @Override
    protected void onProcessGamepadControls(Gamepad gamepad) {

    }
}
