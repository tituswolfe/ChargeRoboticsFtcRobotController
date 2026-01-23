package org.firstinspires.ftc.teamcode.hardware.controllers.subsystems;

import com.pedropathing.control.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.hardware.controllers.motor.VelocityMotorController;

public class DualMotorVelocityController extends VelocityMotorController {
    DcMotorEx secondMotor;

    public DualMotorVelocityController(DcMotorEx device, String name, PIDFCoefficients pidfCoefficients, double ticksPerRevolution, double totalGearRatio, double maxPower, DcMotorEx secondMotor) {
        super(device, name, pidfCoefficients, ticksPerRevolution, totalGearRatio, maxPower);

        this.secondMotor = secondMotor;
    }

    @Override
    public void update() {
        super.update();
        secondMotor.setPower(power);
    }
}
