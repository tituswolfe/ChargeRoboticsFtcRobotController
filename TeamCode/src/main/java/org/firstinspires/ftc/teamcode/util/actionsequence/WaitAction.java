package org.firstinspires.ftc.teamcode.util.actionsequence;

public class WaitAction extends TimedAction{
    public WaitAction(double durationMs) {
        super(() -> {}, durationMs);
    }
}
