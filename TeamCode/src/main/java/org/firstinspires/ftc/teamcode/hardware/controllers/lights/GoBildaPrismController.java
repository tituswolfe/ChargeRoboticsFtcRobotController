package org.firstinspires.ftc.teamcode.hardware.controllers.lights;

import org.firstinspires.ftc.teamcode.hardware.controllers.HardwareController;
import org.firstinspires.ftc.teamcode.hardware.controllers.lights.Prism.GoBildaPrismDriver;
import org.firstinspires.ftc.teamcode.util.actionsequence.Action;
import org.firstinspires.ftc.teamcode.util.actionsequence.ActionSequence;
import org.firstinspires.ftc.teamcode.util.actionsequence.InstantAction;
import org.firstinspires.ftc.teamcode.util.actionsequence.Wait;

public class GoBildaPrismController extends HardwareController<GoBildaPrismDriver> {
    private GoBildaPrismDriver.Artboard activeArtboard;
    private GoBildaPrismDriver.Artboard baseArtboard;
    private GoBildaPrismDriver.Artboard indicateArtboard;
    //ActionSequence indicateSequence;

    long indicateStartTime = -1;
    int indicateDurationMills;

    public GoBildaPrismController(GoBildaPrismDriver device, String name) {
        super(device, name);
    }

    public void setBaseArtboard(GoBildaPrismDriver.Artboard artboard) {
        this.baseArtboard = artboard;
        loadArtboard(artboard);
    }

    private void loadArtboard(GoBildaPrismDriver.Artboard artboard) {
        if (artboard == activeArtboard) {
            return;
        }

        device.loadAnimationsFromArtboard(artboard);
        activeArtboard = artboard;
    }

    @Override
    public void update(long deltaTimeNS) {
        if (indicateStartTime + indicateDurationMills <= System.currentTimeMillis()) {
            loadArtboard(baseArtboard);
            indicateStartTime = -1;
        }
    }

    public void indicate(GoBildaPrismDriver.Artboard artboard) {
        indicate(artboard, 200);
    }

    public void indicate(GoBildaPrismDriver.Artboard artboard, int indicateDurationMills) {
        indicateArtboard = artboard;
        this.indicateDurationMills = indicateDurationMills;
        indicateStartTime = System.currentTimeMillis();
        loadArtboard(artboard);
    }
}
