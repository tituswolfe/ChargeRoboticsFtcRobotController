package org.firstinspires.ftc.teamcode.hardware.controllers.subsystems;

import org.firstinspires.ftc.teamcode.hardware.controllers.motor.TurntablePIDFMotorController;
import org.firstinspires.ftc.teamcode.hardware.controllers.motor.VelocityPIDFMotorController;
import org.firstinspires.ftc.teamcode.hardware.controllers.servo.AngleServoController;
import org.firstinspires.ftc.teamcode.util.math.Angle;

public record Turret<FlywheelController extends VelocityPIDFMotorController>(
        FlywheelController flywheelController, TurntablePIDFMotorController turntableController,
        AngleServoController hoodServoController) {

    public void update(double targetVelRPM, double targetHeading, double targetHoodAngle, long deltaTimeNs) {
        flywheelController.setTargetOutputVelocity(targetVelRPM);
        turntableController.setTargetHeading(targetHeading);
        hoodServoController.setTargetAngle(targetHoodAngle);

        flywheelController.update(deltaTimeNs);
        turntableController.update(deltaTimeNs);
        hoodServoController.update(deltaTimeNs);
    }
}