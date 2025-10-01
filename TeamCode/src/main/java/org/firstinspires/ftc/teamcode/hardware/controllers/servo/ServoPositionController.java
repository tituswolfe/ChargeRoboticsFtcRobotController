package org.firstinspires.ftc.teamcode.hardware.controllers.servo;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoControllerEx;

import java.util.HashMap;

public class ServoPositionController<Position extends Enum> {
    private final Servo servo;
    private final HashMap<Position, Double> positionMap;

    public ServoPositionController(Servo servo, HashMap<Position, Double> positionMap) {
        this.servo = servo;
        this.positionMap = positionMap;
    }

    public void setPosition(Position position) {
        double servoPosition = positionMap.get(position);
        servo.setPosition(servoPosition);
    }

    public Servo getServo() {
        return servo;
    }
}
