package org.firstinspires.ftc.teamcode.hardware.controllers.servo;

import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.Servo;

public class ServoTimerController {
    private final Servo servo;
    private final Timer actionTimer = new Timer();
//    private Timer cooldownTimer = new Timer();
 private int cooldownTimeMills;
    private int transitTimeMills;
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


//    /**
//     * @param position target position
//     * @param transitTimeMills milliseconds to wait before going to resetPosition
//     * @param resetPosition reset position
//     * @param cooldownTimeMills milliseconds to wait after transit, before you can {@link #setPosition(double, int, double, int)} again
//     */
//    public void setPosition(double position, int transitTimeMills, double resetPosition, int cooldownTimeMills) {
//        if (actionTimer.getElapsedTime() < this.transitTimeMills + this.cooldownTimeMills) return;
//
//        servo.setPosition(position);
//        this.transitTimeMills = transitTimeMills;
//        this.cooldownTimeMills = cooldownTimeMills;
//        this.resetPosition = resetPosition;
//
//        actionTimer.resetTimer();
//    }

    public void update() {
        if (transitTimeMills < actionTimer.getElapsedTime() && servo.getPosition() != resetPosition) servo.setPosition(resetPosition);
    }

    public Servo getServo() {
        return servo;
    }
}
