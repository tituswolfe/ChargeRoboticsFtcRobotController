package org.firstinspires.ftc.teamcode.hardware.controllers.servo;

import com.qualcomm.robotcore.hardware.Servo;

import java.util.HashMap;

@Deprecated
public class ServoPositionController<Position extends Enum> {
    private final Servo servo;
    private final HashMap<Position, Double> positionMap;

    public ServoPositionController(Servo servo, HashMap<Position, Double> positionMap) {
        this.servo = servo;
        this.positionMap = positionMap;
    }

    public void setPosition(Position position) {
        if (positionMap == null || positionMap.get(position) == null) return;
        servo.setPosition(positionMap.get(position));
    }

    public void setPosition(double position) {
        servo.setPosition(position);
    }

    public Servo getServo() {
        return servo;
    }
}
