package org.firstinspires.ftc.teamcode.util.actionsequence;

import java.util.function.BooleanSupplier;

public class WaitForConditionAction extends Action{
    public WaitForConditionAction(BooleanSupplier condition) {
        super(() -> {}, condition);
    }
}
