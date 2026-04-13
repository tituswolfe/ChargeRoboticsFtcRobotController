package org.firstinspires.ftc.teamcode.util.actionsequence;

import java.util.function.BooleanSupplier;

public class WaitForCondition extends Action{
    private final BooleanSupplier condition;

    public WaitForCondition(BooleanSupplier condition) {
        super(() -> {});
        this.condition = condition;
    }


    @Override
    public boolean isFinished() {
        return condition.getAsBoolean();
    }
}
