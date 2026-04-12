package org.firstinspires.ftc.teamcode.hardware.controllers.motor;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.util.info.MotorInfo;

public class ThrottleMotorController extends MotorController {
    public ThrottleMotorController(DcMotorEx device, String name, MotorInfo motorInfo, double totalGearRatio, double maxPower) {
        super(device, name, motorInfo, totalGearRatio, maxPower);

        device.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setTargetPower(double power) {
        this.targetPower = power;
    }
}
