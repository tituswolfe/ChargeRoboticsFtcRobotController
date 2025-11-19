package org.firstinspires.ftc.teamcode.hardware.controllers.subsystems;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.hardware.controllers.motor.AngleMotorController;
import org.firstinspires.ftc.teamcode.hardware.controllers.motor.VelocityMotorController;
import org.firstinspires.ftc.teamcode.hardware.controllers.servo.AngleServoController;

public class Turret {
    private final VelocityMotorController flywheelController;
    private final AngleMotorController turntableController;
    private final AngleServoController hoodServoController;

    public Turret(VelocityMotorController flywheelController, AngleMotorController turntableController, AngleServoController hoodServoController) {
        this.flywheelController = flywheelController;
        this.turntableController = turntableController;
        this.hoodServoController = hoodServoController;
    }

    public VelocityMotorController getFlywheelController() {
        return flywheelController;
    }

    public AngleMotorController getTurntableController() {
        return turntableController;
    }
}