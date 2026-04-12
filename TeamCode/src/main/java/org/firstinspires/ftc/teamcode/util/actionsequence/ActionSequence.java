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

//    public void start() {
//        if (actions.length > 0) {
//            currentAction = 0;
//            currentWaitTime = 0;
//            timer.resetTimer();
//        }
//    }

    public void start() {
        if (actions.length > 0) {
            currentAction = 0;
            // Reset the first action so it initializes its timer correctly
            actions[0].reset();
            //timer.resetTimer();
        }
    }


    public void update() {
        if (!isRunning()) return;

        Action activeAction = actions[currentAction];
        activeAction.run();

        if (activeAction.isFinished()) {
            currentAction++;

            // Check if there is a next action to reset
            if (currentAction < actions.length) {
                actions[currentAction].reset();
            }
        }
    }

    public boolean isRunning() {
        // CurrentAction must be less than length to be a valid index
        return currentAction != -1 && currentAction < actions.length;
    }

    public void stop() {
        currentAction = -1;
        currentWaitTime = 0;
    }
}
