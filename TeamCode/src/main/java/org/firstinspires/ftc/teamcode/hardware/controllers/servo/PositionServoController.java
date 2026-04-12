package org.firstinspires.ftc.teamcode.hardware.controllers.servo;

import com.qualcomm.robotcore.hardware.Servo;

public class PositionServoController extends ServoController{
    public PositionServoController(Servo device, String name, double totalRotation, double totalGearRatio) {
        super(device, name, totalRotation, totalGearRatio);
    }

    public void setPosition(double position) {
        sendPosition(position);
    }

    @Override
    public void update(long deltaTimeNS) {

    }
}
