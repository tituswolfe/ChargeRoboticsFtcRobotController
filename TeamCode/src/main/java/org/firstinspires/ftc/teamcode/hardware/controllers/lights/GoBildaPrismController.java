package org.firstinspires.ftc.teamcode.hardware.controllers.lights;

import com.pedropathing.util.Timer;

import org.firstinspires.ftc.teamcode.hardware.controllers.HardwareController;
import org.firstinspires.ftc.teamcode.hardware.controllers.lights.Prism.Color;
import org.firstinspires.ftc.teamcode.hardware.controllers.lights.Prism.GoBildaPrismDriver;
import org.firstinspires.ftc.teamcode.hardware.controllers.lights.Prism.PrismAnimations;
import org.firstinspires.ftc.teamcode.util.actionsequence.Action;
import org.firstinspires.ftc.teamcode.util.actionsequence.ActionSequence;
import org.firstinspires.ftc.teamcode.util.actionsequence.InstantAction;
import org.firstinspires.ftc.teamcode.util.actionsequence.WaitAction;

public class GoBildaPrismController extends HardwareController<GoBildaPrismDriver> {
    private PrismAnimations.AnimationBase prismAnimation;
    private Color indicateColor = Color.TRANSPARENT;
    private int indicateTimeMills = 0;

    Action[] indicateActions = new Action[] {
            new InstantAction(() -> device.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_0, new PrismAnimations.Solid(indicateColor))),
            new WaitAction(indicateTimeMills),
            new InstantAction(() -> device.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_0, prismAnimation))

    };
    ActionSequence indicateSequence = new ActionSequence(indicateActions);

    public GoBildaPrismController(GoBildaPrismDriver device, String name) {
        super(device, name);
    }

    @Override
    public void start() {

    }

    @Override
    public void update(long deltaTimeNS) {
    }

    public void indicate(Color indicateColor) {
        indicate(indicateColor, 300);
    }

    public void indicate(Color indicateColor, int indicateTimeMills) {
        this.indicateColor = indicateColor;
        this.indicateTimeMills = indicateTimeMills;

        indicateSequence.start();
    }
}
