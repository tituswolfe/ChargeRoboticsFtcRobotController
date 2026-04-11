package org.firstinspires.ftc.teamcode.util.actionsequence;

import java.util.function.BooleanSupplier;

public class Action {
    private final Runnable function;
    private final BooleanSupplier condition;

    public Action(Runnable function, BooleanSupplier condition) {
        this.function = function;
        this.condition = condition;
    }

    public boolean isFinished() {
        return condition.getAsBoolean();
    }

    public void run() {
        function.run();
    }
}
