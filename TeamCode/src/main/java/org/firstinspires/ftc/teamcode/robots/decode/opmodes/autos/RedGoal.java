package org.firstinspires.ftc.teamcode.robots.decode.opmodes.autos;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.robots.base.opmodes.BaseAuto;
import org.firstinspires.ftc.teamcode.robots.decode.DecodeGamepadController;
import org.firstinspires.ftc.teamcode.robots.decode.DecodeRobot;
import org.firstinspires.ftc.teamcode.util.ThreadUtil;

@Autonomous(name = "Red Goal", group = "decode", preselectTeleOp = "Decode")
public class RedGoal extends BaseAuto<DecodeRobot, DecodeGamepadController, DecodeGamepadController> {

    @Override
    public void autonomousPathUpdate(int pathState) {
        switch (pathState) {
            case 0:
                robot.startFlywheels();
                robot.runIntake();
                robot.getFollower().followPath(robot.shoot1, true);
                setPathState(1);
                break;
            case 1:

                if (robot.getFollower().atPose(robot.closeShoot, 1, 1)) {
                    robot.launchArtifact();
                    ThreadUtil.sleep(1000);
                    robot.launchArtifact();
                    ThreadUtil.sleep(1000);
                    robot.launchArtifact();

                    ThreadUtil.sleep(1000);

                    setPathState(2);
                }
                break;
            case 2:
                robot.getFollower().holdPoint(new Pose(0, 0,0));
                setPathState(-1);

                break;

        }
    }

    @Override
    protected DecodeRobot instantiateRobot() {
        return new DecodeRobot();
    }

    @Override
    protected Pose instantiateStartPose() {
        return robot.autoStart;
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
