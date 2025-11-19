package org.firstinspires.ftc.teamcode.hardware.controllers.servo;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.math.Angle;

public class ServoController {
    protected final Servo servo;
    public final Angle maxRotation;
    // TODO: Total gear ratio

    public ServoController(Servo servo, Angle maxRotation) {
        this.servo = servo;
        this.maxRotation = maxRotation;
    }

    public Servo getServo() {
        return servo;
    }
}
