package org.firstinspires.ftc.teamcode.robots.drivechassis.opmodes;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.robots.base.opmodes.BaseOpMode;
import org.firstinspires.ftc.teamcode.robots.drivechassis.DriveGamepadHandler;
import org.firstinspires.ftc.teamcode.robots.drivechassis.DriveRobot;

@Autonomous(name = "Drive Auto", preselectTeleOp = "Drive")
public class DriveAuto extends BaseOpMode<DriveRobot, DriveGamepadHandler, DriveGamepadHandler> {
    Pose startPose = new Pose(0, 0, 0);
    Pose secondPose = new Pose(12, 0, 0);

    PathChain path;
    @Override
    public void init() {
        super.init();
    }

    @Override
    protected void adjustHardwareBeforeStart() {

    }

    @Override
    protected void buildPaths() {
        path = robot.getFollower().pathBuilder()
                .addPath(new BezierLine(startPose, secondPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), secondPose.getHeading())
                .setTangentHeadingInterpolation()
                .build();
    }

    @Override
    public void start() {


    }

    @Override
    public void loop() {
        robot.getFollower().followPath(path);

        super.loop();
    }

    @Override
    protected void autonomousPathUpdate(int pathState) {
        switch (pathState) {
            case 0:
                robot.getFollower().followPath(path);
                setPathState(1);
                break;
            case 1:

            /* You could check for
            - Follower State: "if(!follower.isBusy()) {}"
            - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
            - Robot Position: "if(follower.getPose().getX() > 36) {}"
            */

                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!robot.getFollower().isBusy()) {
                    /* Score Preload */

//                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
//                    follower.followPath(grabPickup1,true);
                    setPathState(2);
                }
                break;

            case 2:
//                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
//                if(!follower.isBusy()) {
//                    /* Set the state to a Case we won't use or define, so it just stops running an new paths */
//                    setPathState(-1);
//                }
//                break;
        }
    }

    @Override
    protected DriveRobot instantiateRobot() {
        return new DriveRobot();
    }

    @Override
    protected Pose instantiateStartPose() {
        return null;
    }

    @Override
    protected DriveGamepadHandler instantiateGamepadHandler1() {
        return new DriveGamepadHandler(robot, this, gamepad1, true);
    }

    @Override
    protected DriveGamepadHandler instantiateGamepadHandler2() {
        return new DriveGamepadHandler(robot, this, gamepad1, true);
    }

    @Override
    protected AlliancePosition instantiateAlliancePosition() {
        return null;
    }

    @Override
    protected FieldType instantiateFieldType() {
        return null;
    }
}
