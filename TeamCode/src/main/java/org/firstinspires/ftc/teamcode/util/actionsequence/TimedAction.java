package org.firstinspires.ftc.teamcode.util.actionsequence;

import com.pedropathing.util.Timer;

public class TimedAction extends Action {
    private final Timer actionTimer;

    private TimedAction(Runnable function, double durationMs, Timer actionTimer) {
        super(function, () -> actionTimer.getElapsedTime() > durationMs);
        this.actionTimer = actionTimer;
    }

    public TimedAction(Runnable function, double durationMs) {
        this(function, durationMs, new Timer());
    }

    @Override
    public void run() {
        actionTimer.resetTimer();
        super.run();
    }
}
