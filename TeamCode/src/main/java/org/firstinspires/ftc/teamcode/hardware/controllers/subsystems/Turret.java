package org.firstinspires.ftc.teamcode.hardware.controllers.subsystems;

import org.firstinspires.ftc.teamcode.hardware.controllers.motor.TurntableMotorController;
import org.firstinspires.ftc.teamcode.hardware.controllers.motor.VelocityMotorController;
import org.firstinspires.ftc.teamcode.hardware.controllers.servo.AngleServoController;
import org.firstinspires.ftc.teamcode.util.math.Angle;

public record Turret(VelocityMotorController flywheelController,
                     TurntableMotorController turntableController,
                     AngleServoController hoodServoController) {

    public void update(double targetVelRPM, Angle targetTurntableHeading, Angle targetHoodAngle) {
        flywheelController.setTargetVelocity(targetVelRPM);
        turntableController.setTargetHeading(targetTurntableHeading);
        // turntableController.getPidfController().updateFeedForwardInput();
        hoodServoController.setTargetAngle(targetHoodAngle);

        flywheelController.update();
        turntableController.update();
    }
}