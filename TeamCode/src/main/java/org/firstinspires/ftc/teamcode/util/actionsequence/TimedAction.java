package org.firstinspires.ftc.teamcode.util.actionsequence;

import com.pedropathing.util.Timer;

public class TimedAction extends Action {
    private final double durationMs;
    private long startTime = -1;

    public TimedAction(Runnable function, double durationMs) {
        super(function);
        this.durationMs = durationMs;
    }

    @Override
    public void reset() {
        startTime = -1;
    }

    @Override
    public void run() {
        if (startTime == -1) {
            startTime = System.currentTimeMillis();
            super.run(); // Run the actual task
        }
    }

    @Override
    public boolean isFinished() {
        if (startTime == -1) return false;
        return (System.currentTimeMillis() - startTime) >= durationMs;
    }
}