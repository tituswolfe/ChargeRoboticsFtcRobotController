package org.firstinspires.ftc.teamcode.robots.jetfire.opmodes.autos;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.robots.base.opmodes.BaseAuto;
import org.firstinspires.ftc.teamcode.robots.jetfire.JetFireRobot;

@Autonomous(name = "Blue Goal", group = "decode", preselectTeleOp = "Decode")
public class BlueGoal extends BaseAuto<JetFireRobot> {

    @Override
    public void autonomousPathUpdate(int pathState) {
        switch (pathState) {
            case 0:
                robot.startFlywheels(0.9);
                robot.startIntake();

                robot.getFollower().followPath(robot.shoot1, true);
                setPathState(1);
                break;
            case 1:
                if (!robot.getFollower().isBusy()) {
                    robot.launchArtifact();
                    activeSleep(1000);
                    robot.launchArtifact();
                    activeSleep(1000);
                    robot.launchArtifact();
                    activeSleep(2000);

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
    protected JetFireRobot instantiateRobot() {
        return new JetFireRobot();
    }

    @Override
    protected Pose instantiateStartPose() {
        return robot.blueAutoStart;
    }

    @Override
    protected AllianceColor instantiateAllianceColor() {
        return AllianceColor.BLUE;
    }
}
