package org.firstinspires.ftc.teamcode.robots.standard;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.robots.base.BaseOpMode;

public class StandardOpMode extends BaseOpMode<StandardRobot, StandardGamepadController> {
    @Override
    public void stop() {
        robot.getDriveTrain().disable();
        super.stop();
    }


}
