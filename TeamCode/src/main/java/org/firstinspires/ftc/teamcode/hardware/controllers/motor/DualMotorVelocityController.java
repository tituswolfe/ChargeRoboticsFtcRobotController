package org.firstinspires.ftc.teamcode.hardware.controllers.motor;

import com.pedropathing.control.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class DualMotorVelocityController extends VelocityMotorController {
    DcMotorEx secondMotor;

    public DualMotorVelocityController(DcMotorEx device, DcMotorEx secondMotor, String name, PIDFCoefficients pidfCoefficients, double ticksPerRevolution, double totalGearRatio, double maxPower) {
        super(device, name, pidfCoefficients, ticksPerRevolution, totalGearRatio, maxPower);

        this.secondMotor = secondMotor;
    }

    @Override
    public void update() {
        super.update();
        secondMotor.setPower(power);
    }
}
