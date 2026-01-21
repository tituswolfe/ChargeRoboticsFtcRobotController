package org.firstinspires.ftc.teamcode.hardware.controllers.servo;

import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.Servo;

public class ServoTimerController {
    private final Servo servo;
    private final Timer actionTimer = new Timer();
//    private Timer cooldownTimer = new Timer();
    private int transitTimeMills = 0;
    private double resetPosition;

    public ServoTimerController(Servo servo, double initPos) {
        this.servo = servo;
        this.servo.setPosition(initPos);
        resetPosition = initPos;
    }

    public void setPosition(double position, int millsToWait, double resetPosition) {
        servo.setPosition(position);
        this.transitTimeMills = millsToWait;
        this.resetPosition = resetPosition;
        actionTimer.resetTimer();
    }

    public void update() {
        if (transitTimeMills < actionTimer.getElapsedTime() && servo.getPosition() != resetPosition){
            servo.setPosition(resetPosition);
        }
    }

    public void setPosition(double position) {
        servo.setPosition(position);
        this.transitTimeMills = 0;
        this.resetPosition = position;
        actionTimer.resetTimer();
    }

    public Servo getServo() {
        return servo;
    }
}
