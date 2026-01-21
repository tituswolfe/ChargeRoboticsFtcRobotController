package org.firstinspires.ftc.teamcode.hardware.controllers.subsystems;

import com.pedropathing.control.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.hardware.controllers.BasicMotorController;
import org.firstinspires.ftc.teamcode.hardware.controllers.VelocityMotorController;

public class DualMotorVelocityController extends VelocityMotorController {
    DcMotorEx secondMotor;

    public DualMotorVelocityController(DcMotorEx device, String name, DcMotorSimple.Direction direction, PIDFCoefficients pidfCoefficients, double ticksPerRevolution, double totalGearRatio, double maxPower, DcMotorEx secondMotor) {
        super(device, name, direction, pidfCoefficients, ticksPerRevolution, totalGearRatio, maxPower);

        this.secondMotor = secondMotor;
    }

    @Override
    public void update(long deltaTime) {
        super.update(deltaTime);

        secondMotor.setPower(power);
    }
}
