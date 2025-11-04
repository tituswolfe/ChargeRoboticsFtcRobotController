package org.firstinspires.ftc.teamcode.robots.base.opmodes;

import org.firstinspires.ftc.teamcode.robots.base.GamepadHandlerBase;
import org.firstinspires.ftc.teamcode.robots.base.RobotBase;
import org.firstinspires.ftc.teamcode.robots.base.StaticData;

public abstract class TeleOpBase<Robot extends RobotBase> extends OpModeBase<Robot> {
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
