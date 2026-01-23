package org.firstinspires.ftc.teamcode.hardware.controllers.subsystems;

import org.firstinspires.ftc.teamcode.robots.season.decode.jetfire.JetfireRobot;

public class MotorisedIntake {
    private final VelocityMotorController velocityMotorController;


    public enum IntakeMode {
        INTAKE,
        REVERSE,
        OFF
    }
    private JetfireRobot.IntakeMode intakeMode;

    public MotorisedIntake(VelocityMotorController velocityMotorController) {
        this.velocityMotorController = velocityMotorController;
    }

    public void setIntakeMode(JetfireRobot.IntakeMode intakeMode) {
        this.intakeMode = intakeMode;
    }
}
