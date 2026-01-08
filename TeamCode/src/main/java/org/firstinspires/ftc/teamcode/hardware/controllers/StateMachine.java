package org.firstinspires.ftc.teamcode.hardware.controllers;

import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;

public class StateMachine {
    private static class Step {
        BooleanSupplier condition;
        Runnable action;

        public Step(BooleanSupplier condition, Runnable action) {
            this.condition = condition;
            this.action = action;
        }
    }

    private final List<Step> steps = new ArrayList<>();
    private int currentIndex = -1;
    private boolean running = false;

    public void addStep(BooleanSupplier condition, Runnable action) {
        steps.add(new Step(condition, action));
    }

    public void start() {
        if (steps.isEmpty()) return;

        currentIndex = 0;
        running = true;
    }

    public void update() {
        if (!running || isFinished()) return;

        Step currentStep = steps.get(currentIndex);

        if (currentStep.condition.getAsBoolean()) {
            currentStep.action.run();
            currentIndex++;
        }
    }

    public void reset() {
        currentIndex = -1;
        running = false;
    }

    public boolean isFinished() {
        return currentIndex >= steps.size();
    }

    // TODO: Rely on running or isFinished?
}
