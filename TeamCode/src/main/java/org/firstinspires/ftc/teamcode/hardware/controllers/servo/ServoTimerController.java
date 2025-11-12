package org.firstinspires.ftc.teamcode.hardware.controllers.servo;

import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.Servo;

public class ServoTimerController {
    private final Servo servo;
    private final Timer actionTimer = new Timer();
    private int millsToWait;
    private double resetPosition;

    public ServoTimerController(Servo servo) {
        this.servo = servo;
    }

    public void setPosition(double position, int millsToWait, double resetPosition) {
        servo.setPosition(position);
        this.millsToWait = millsToWait;
        this.resetPosition = resetPosition;
        actionTimer.resetTimer();
    }

    public void update() {
        if (millsToWait < actionTimer.getElapsedTime() && servo.getPosition() != resetPosition) servo.setPosition(resetPosition);
    }

    public Servo getServo() {
        return servo;
    }
}
