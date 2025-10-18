package org.firstinspires.ftc.teamcode.robots.base.opmodes;

import org.firstinspires.ftc.teamcode.robots.base.GamepadHandlerBase;
import org.firstinspires.ftc.teamcode.robots.base.RobotBase;
import org.firstinspires.ftc.teamcode.robots.base.StaticData;

public abstract class TeleOpBase<Robot extends RobotBase, GamepadHandler1 extends GamepadHandlerBase, GamepadHandler2 extends GamepadHandlerBase> extends OpModeBase<Robot, GamepadHandler1, GamepadHandler2> {
    @Override
    protected AllianceColor instantiateAllianceColor() {
        if (StaticData.lastAllianceColor != null) {
            return StaticData.lastAllianceColor;
        }
        return null;
    }

    @Override
    protected OpModeType instantiateOpModeType() {
        return OpModeType.TELEOP;
    }
}
