package org.firstinspires.ftc.teamcode.util.actionsequence;

import com.pedropathing.util.Timer;

public class ActionSequence {
    private final Action[] actions;
    private int currentAction = -1;

    public ActionSequence(Action[] actions) {
        this.actions = actions;
    }

    public void start() {
        if (actions.length > 0) {
            currentAction = 0;
            actions[0].reset();
        }
    }

    public void update() {
        if (!isRunning()) return;

        Action activeAction = actions[currentAction];
        activeAction.run();

        if (activeAction.isFinished()) {
            currentAction++;

            if (currentAction < actions.length) {
                actions[currentAction].reset();
            }
        }
    }

    public boolean isRunning() {
        return currentAction != -1 && currentAction < actions.length;
    }

    public void stop() {
        currentAction = -1;
    }
}
