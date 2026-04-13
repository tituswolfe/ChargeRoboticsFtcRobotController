package org.firstinspires.ftc.teamcode.hardware.controllers.lights;

import org.firstinspires.ftc.teamcode.hardware.controllers.HardwareController;
import org.firstinspires.ftc.teamcode.hardware.controllers.lights.Prism.GoBildaPrismDriver;
import org.firstinspires.ftc.teamcode.util.actionsequence.Action;
import org.firstinspires.ftc.teamcode.util.actionsequence.ActionSequence;
import org.firstinspires.ftc.teamcode.util.actionsequence.InstantAction;
import org.firstinspires.ftc.teamcode.util.actionsequence.Wait;

public class GoBildaPrismController extends HardwareController<GoBildaPrismDriver> {
    private GoBildaPrismDriver.Artboard currentArtboard;
    //private GoBildaPrismDriver.Artboard indicateArtboard;
    //private int indicateTimeMills;

    ActionSequence indicateSequence;

    public GoBildaPrismController(GoBildaPrismDriver device, String name) {
        super(device, name);
    }

    public void loadArtboard(GoBildaPrismDriver.Artboard artboard) {
        device.loadAnimationsFromArtboard(artboard);
        currentArtboard = artboard;
    }

    @Override
    public void update(long deltaTimeNS) {
        if (indicateSequence != null) {
            indicateSequence.update();
        }
    }

    public void indicate(GoBildaPrismDriver.Artboard artboard) {
        indicate(artboard, 200);
    }

    public void indicate(GoBildaPrismDriver.Artboard artboard, int indicateTimeMills) {
        if (indicateSequence != null) {
            if (indicateSequence.isRunning()) {
                return;
            }
        }



        Action[] indicateActions = new Action[] {
                new InstantAction(() -> device.loadAnimationsFromArtboard(artboard)),
                new Wait(indicateTimeMills),
                new InstantAction(() -> device.loadAnimationsFromArtboard(currentArtboard))

        };
        indicateSequence = new ActionSequence(indicateActions);
//
//        this.indicateArtboard = artboard;
//        this.indicateTimeMills = indicateTimeMills;

        indicateSequence.start();
    }
}
