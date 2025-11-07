package org.firstinspires.ftc.teamcode.hardware.controllers.servo;

import com.qualcomm.robotcore.hardware.Servo;

public class RGBIndicatorLightController {
    private final Servo indicatorLight;
    
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
    Color color;
    
    public RGBIndicatorLightController(Servo indicatorLight) {
        this.indicatorLight = indicatorLight;
        this.indicatorLight.setPosition(0);
        color = Color.OFF;
    }

// https://www.gobilda.com/rgb-indicator-light-pwm-controlled/
    public void setColor(Color color) {
        if (this.color == color) return;
        this.color = color;

        indicatorLight.setPosition(switch (color) {
            case OFF -> 0;
            case RED -> 0.277;
            case ORANGE -> 0.333;
            case YELLOW -> 0.388;
            case SAGE -> 0.444;
            case GREEN -> 0.5;
            case AZURE -> 0.555;
            case BLUE -> 0.611;
            case INDIGO -> 0.666;
            case VIOLET -> 0.722;
            case WHITE -> 1;
        });
    }
}
