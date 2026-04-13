package org.firstinspires.ftc.teamcode.util.actionsequence;

import java.util.function.BooleanSupplier;

public abstract class Action {
    private final Runnable function;

    public Action(Runnable function) {
        this.function = function;
    }

    public abstract boolean isFinished();

    public void run() {
        function.run();
    }

    public void reset() {}
}
