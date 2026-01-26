package org.firstinspires.ftc.teamcode.hardware.controllers.servo;

import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.hardware.controllers.HardwareController;

public class RGBIndicatorLightController extends HardwareController<Servo> {
    public enum Color {
        OFF,
        RED,
        ORANGE,
        YELLOW,
        SAGE,
        GREEN,
        AZURE,
        BLUE,
        INDIGO,
        VIOLET,
        WHITE
    }
    Color color = Color.OFF;

    Timer indicateTimer = new Timer();
    private static final int INDICATE_TIME_MS = 500;
    RGBIndicatorLightController.Color indicateColor = RGBIndicatorLightController.Color.OFF;

    public RGBIndicatorLightController(Servo device, String name) {
        super(device, name);
        device.setPosition(0);
    }

    @Override
    public void update() {
        if (indicateTimer.getElapsedTime() < INDICATE_TIME_MS) {
            setColor(indicateColor);
        }
    }

    // https://www.gobilda.com/rgb-indicator-light-pwm-controlled/
    public void setColor(Color color) {
        if (this.color == color) return;
        this.color = color;

        device.setPosition(switch (color) {
            case OFF -> 0;
            case RED -> 0.28; // RED position increased due to PWM variation, turning it OFF
            case ORANGE -> 0.333;
            case YELLOW -> 0.388;
            case SAGE -> 0.444;
            case GREEN -> 0.5;
            case AZURE -> 0.555;
            case BLUE -> 0.611;
            case INDIGO -> 0.666;
            case VIOLET -> 0.719;
            case WHITE -> 1;
        });
    }

    public void indicate(Color color) {
        indicateColor = color;
        indicateTimer.resetTimer();
    }
}
