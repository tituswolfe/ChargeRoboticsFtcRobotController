package org.firstinspires.ftc.teamcode.hardware.controllers.motor;

import com.pedropathing.control.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.util.info.MotorInfo;

public class DualPIDFMotorVelocityController extends VelocityPIDFMotorController {
    DcMotorEx secondMotor;

    public DualPIDFMotorVelocityController(DcMotorEx firstMotor, DcMotorEx secondMotor, String name, MotorInfo motorInfo, double totalGearRatio, double maxPower, PIDFCoefficients pidfCoefficients) {
        super(firstMotor, name, motorInfo, totalGearRatio, maxPower, pidfCoefficients);

        this.secondMotor = secondMotor;
    }


    @Override
    protected void sendPowerToMotor(double power) {
        secondMotor.setPower(power);
        super.sendPowerToMotor(power);
    }
}
