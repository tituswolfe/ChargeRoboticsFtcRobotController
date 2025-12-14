package org.firstinspires.ftc.teamcode.hardware.controllers.subsystems;

import org.firstinspires.ftc.teamcode.hardware.controllers.motor.VelocityMotorController;
import org.firstinspires.ftc.teamcode.robots.season.decode.dug.JetFireRobot;

public class MotorisedIntake {
    private final VelocityMotorController velocityMotorController;


    public enum IntakeMode {
        INTAKE,
        REVERSE,
        OFF
    }
    private JetFireRobot.IntakeMode intakeMode;

    public MotorisedIntake(VelocityMotorController velocityMotorController) {
        this.velocityMotorController = velocityMotorController;
    }

    public void setIntakeMode(JetFireRobot.IntakeMode intakeMode) {
        this.intakeMode = intakeMode;
    }
}
