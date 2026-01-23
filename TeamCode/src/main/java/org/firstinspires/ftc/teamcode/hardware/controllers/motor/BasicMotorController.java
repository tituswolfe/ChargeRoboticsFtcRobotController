package org.firstinspires.ftc.teamcode.hardware.controllers.motor;

import com.pedropathing.control.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class BasicMotorController extends MotorController {
    public BasicMotorController(DcMotorEx device, String name, DcMotorSimple.Direction direction, PIDFCoefficients pidfCoefficients, double ticksPerRevolution, double totalGearRatio, double maxPower) {
        super(device, name, direction, pidfCoefficients, ticksPerRevolution, totalGearRatio, maxPower);
    }
}
