package org.firstinspires.ftc.teamcode.robots.decode.opmodes.autos;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.robots.base.opmodes.BaseAuto;
import org.firstinspires.ftc.teamcode.robots.decode.DecodeGamepadController;
import org.firstinspires.ftc.teamcode.robots.decode.DecodeRobot;

@Autonomous(name = "Red Goal", group = "decode", preselectTeleOp = "Decode")
public class RedGoal extends BaseAuto<DecodeRobot, DecodeGamepadController, DecodeGamepadController> {

    @Override
    public int autonomousPathUpdate(int pathState) {
        return 0;
    }

    @Override
    protected DecodeRobot instantiateRobot() {
        return new DecodeRobot();
    }

    @Override
    protected Pose instantiateStartPose() {
        return new Pose(0 , 0, 0);
    }

    @Override
    protected DecodeGamepadController instantiateGamepadHandler1() {
        return null;
    }

    @Override
    protected DecodeGamepadController instantiateGamepadHandler2() {
        return null;
    }

    @Override
    protected AllianceColor instantiateAllianceColor() {
        return AllianceColor.RED;
    }
}
