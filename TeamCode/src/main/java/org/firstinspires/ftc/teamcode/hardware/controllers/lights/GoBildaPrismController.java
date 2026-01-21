package org.firstinspires.ftc.teamcode.hardware.controllers.lights;

import com.pedropathing.util.Timer;

import org.firstinspires.ftc.teamcode.hardware.controllers.HardwareController;
import org.firstinspires.ftc.teamcode.hardware.controllers.lights.Prism.Color;
import org.firstinspires.ftc.teamcode.hardware.controllers.lights.Prism.GoBildaPrismDriver;
import org.firstinspires.ftc.teamcode.hardware.controllers.lights.Prism.PrismAnimations;

public class GoBildaPrismController extends HardwareController<GoBildaPrismDriver> {
    private Color indicateColor = Color.TRANSPARENT;
    private int indicateTimeMills = 0;
    boolean doneIndicating = true;
    private final Timer indicateTimer = new Timer();

    public GoBildaPrismController(GoBildaPrismDriver device, String name) {
        super(device, name);
    }


    @Override
    public void update(long deltaTime) {
        if (indicateTimer.getElapsedTime() > indicateTimeMills && !doneIndicating) {
            device.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_0, new PrismAnimations.Solid(indicateColor));
            doneIndicating = true;
        }
    }
    // TODO TURRET PRISM LIGHTS??


    public void indicate(Color indicateColor) {
        indicate(indicateColor, 300);
    }

    public void indicate(Color indicateColor, int indicateTimeMills) {
        this.indicateColor = indicateColor;
        this.indicateTimeMills = indicateTimeMills;

        doneIndicating = false;
        indicateTimer.resetTimer();
    }
}
