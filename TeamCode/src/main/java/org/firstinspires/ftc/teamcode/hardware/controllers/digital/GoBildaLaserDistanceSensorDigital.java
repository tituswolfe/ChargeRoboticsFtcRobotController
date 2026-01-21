package org.firstinspires.ftc.teamcode.hardware.controllers.digital;

import com.qualcomm.robotcore.hardware.DigitalChannel;

public class GoBildaLaserDistanceSensorDigital {
    private final DigitalChannel laserInput;

    public GoBildaLaserDistanceSensorDigital(DigitalChannel laserInput) {
        this.laserInput = laserInput;
        laserInput.setMode(DigitalChannel.Mode.INPUT);
    }

    public void update() {

    }

    public boolean isObjectDetected() {
        return laserInput.getState();
    }
}
