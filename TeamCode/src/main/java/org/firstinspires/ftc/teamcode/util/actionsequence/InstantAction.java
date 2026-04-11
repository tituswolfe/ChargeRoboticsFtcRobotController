package org.firstinspires.ftc.teamcode.util.actionsequence;

import java.util.function.BooleanSupplier;

public class InstantAction extends Action {

    public InstantAction(Runnable function) {
        super(function, () -> true);
    }
}
