package org.firstinspires.ftc.teamcode.hardware.controllers.motor;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.util.info.MotorInfo;

public class BasicMotorController extends MotorController{
    public BasicMotorController(DcMotorEx device, String name, MotorInfo motorInfo, double totalGearRatio, double maxPower) {
        super(device, name, motorInfo, totalGearRatio, maxPower);
    }

    public void setTargetPower(double power) {
        this.targetPower = power;
    }
}
