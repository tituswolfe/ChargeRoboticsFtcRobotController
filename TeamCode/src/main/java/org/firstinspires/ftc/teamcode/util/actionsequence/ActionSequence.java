package org.firstinspires.ftc.teamcode.util.actionsequence;

import com.pedropathing.util.Timer;

public class ActionSequence {
    private final Action[] actions;
    private int currentAction = -1;
    private final Timer timer = new Timer();
    private double currentWaitTime = 0;

    public ActionSequence(Action[] actions) {
        this.actions = actions;
    }

    public void start() {
        if (actions.length > 0) {
            currentAction = 0;
            currentWaitTime = 0;
            timer.resetTimer();
        }
    }

    public void update() {
        if (!isRunning()) return;

        Action activeAction = actions[currentAction];

        if (timer.getElapsedTime() == 0) {
            activeAction.run();
        }

        if (activeAction.isFinished()) {
            currentAction++;
            timer.resetTimer();
        }
    }

    public void stop() {
        currentAction = -1;
        currentWaitTime = 0;
    }

    public boolean isRunning() {
        return currentAction != -1 && currentAction <= actions.length;
    }
}
