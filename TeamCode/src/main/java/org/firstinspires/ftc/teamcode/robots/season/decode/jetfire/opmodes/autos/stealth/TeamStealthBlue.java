package org.firstinspires.ftc.teamcode.robots.season.decode.jetfire.opmodes.autos.stealth;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.robots.base.opmodes.BaseAuto;
import org.firstinspires.ftc.teamcode.robots.season.decode.jetfire.JetfireRobot;

@Autonomous(preselectTeleOp = "Jetfire")
public class TeamStealthBlue extends BaseAuto<JetfireRobot> {
    Pose start = new Pose(63, -14.1, Math.toRadians(-180));
    Pose shootPreload = new Pose(53, -17, Math.toRadians(-159.1));

    Pose line3Control = new Pose(37, -21);
    Pose line3End = new Pose(33, -43, Math.toRadians(-90));

    Pose shoot = new Pose(49.7, -16.3, Math.toRadians(-90));
    Pose intakeLoadingZoneBetween = new Pose(63, -20, Math.toRadians(-90));
    Pose intakeLoadingZoneEnd = new Pose(63, -57, Math.toRadians(-90));
    Pose intakeTunnelEnd = new Pose(63 - 16, -57, Math.toRadians(-90));

    Pose end = new Pose(47, -18, Math.toRadians(-135));

    PathChain startToShootPreload;
    PathChain shootPreloadToIntakeLine3;
    PathChain intakeLine3ToShoot;

    PathChain shootToIntakeLoadingZone;
    PathChain intakeLoadingZoneToShoot;

    PathChain shootToIntakeTunnel;
    PathChain intakeTunnelToShoot;

    int returnPathState = 0;
    public static double endPathTValue = 0.97;

    @Override
    public void autonomousPathUpdate(int pathState) {
        if (opmodeTimer.getElapsedTimeSeconds() >= 28 && pathState > 0) {
            setPathState(-1, true);
            return;
        }

        switch (pathState) {
            case 0:
                robot.getFollower().followPath(startToShootPreload);
                robot.toggleSubsystems(true);
                setPathState(1, true);
                break;
            case 1:
                // SHOOT
                if (actionTimer.getElapsedTime() > 800 && robot.isFlywheelReady()) {
                    returnPathState = 2;
                    setPathState(20, true);
                }
                break;
            case 2:
                robot.getFollower().followPath(shootPreloadToIntakeLine3);
                setPathState(3, true);
                break;
            case 3:
                nextPath(intakeLine3ToShoot, 4);
                break;
            case 4:
                if (robot.getFollower().getCurrentTValue() >= endPathTValue) {
                    returnPathState = 5;
                    setPathState(20, true);
                }
                break;
            case 5:
                robot.getFollower().followPath(shootToIntakeLoadingZone);
                setPathState(6, true);
                break;
            case 6:
                nextPath(intakeLoadingZoneToShoot, 7);
                break;
            case 7:
                if (robot.getFollower().getCurrentTValue() >= endPathTValue) {
                    returnPathState = 8;
                    setPathState(20, true);
                }
                break;
            case 8:
                robot.getFollower().followPath(shootToIntakeTunnel);
                setPathState(9, true);
                break;
            case 9:
                nextPath(intakeTunnelToShoot, 4);
                break;

            case -1:
                if(!robot.getFollower().isBusy()) {
                    robot.toggleSubsystems(false);

                    robot.getFollower().holdPoint(end);
                    setPathState(-2, true);
                }
                break;

            // SHOOT ARTIFACTS
            case 20:
                if (!robot.isArtifactLoaded()) {
                    setPathState(returnPathState, true);
                    break;
                }

                if (robot.isReadyToShoot() && actionTimer.getElapsedTime() > 500) {
                    robot.fire();
                    setPathState(21, true);
                }
                break;
            case 21:
                if (robot.isReadyToShoot()) {
                    robot.fire();
                    setPathState(22, true);
                }
                break;
            case 22:
                if (robot.isReadyToShoot()) {
                    robot.fire();
                    setPathState(23, true);
                }
                break;
            case 23:
                if (actionTimer.getElapsedTime() > 300) {
                    setPathState(returnPathState, true);
                }
                break;
        }
    }

    @Override
    public void buildPaths(Follower follower) {
        startToShootPreload = follower.pathBuilder()
                .addPath(new BezierLine(start, shootPreload))
                .setLinearHeadingInterpolation(start.getHeading(), shootPreload.getHeading())
                .setBrakingStrength(0.1)
                .setBrakingStart(2)
                .build();

        shootPreloadToIntakeLine3 = follower.pathBuilder()
                .addPath(new BezierCurve(shootPreload, line3Control, line3End))
                .setTangentHeadingInterpolation()
                .build();

        intakeLine3ToShoot = follower.pathBuilder()
                .addPath(new BezierLine(line3End, shoot))
                .setLinearHeadingInterpolation(line3End.getHeading(), shoot.getHeading())
                .setBrakingStrength(0.1)
                .setBrakingStart(2)
                .build();

        shootToIntakeLoadingZone = follower.pathBuilder()
                .addPath(new BezierLine(shoot, intakeLoadingZoneBetween))
                .setLinearHeadingInterpolation(shoot.getHeading(), intakeLoadingZoneBetween.getHeading())
                .addPath(new BezierLine(intakeLoadingZoneBetween, intakeLoadingZoneEnd))
                .setLinearHeadingInterpolation(intakeLoadingZoneBetween.getHeading(), intakeLoadingZoneEnd.getHeading())
                .build();

        intakeLoadingZoneToShoot = follower.pathBuilder()
                .addPath(new BezierLine(intakeLoadingZoneEnd, shoot))
                .setLinearHeadingInterpolation(intakeLoadingZoneEnd.getHeading(), shoot.getHeading())
                .setBrakingStrength(0.1)
                .setBrakingStart(2)
                .build();

        shootToIntakeTunnel = follower.pathBuilder()
                .addPath(new BezierLine(shoot, intakeTunnelEnd))
                .setLinearHeadingInterpolation(shoot.getHeading(), intakeTunnelEnd.getHeading())
                .build();

        intakeTunnelToShoot = follower.pathBuilder()
                .addPath(new BezierLine(intakeTunnelEnd, shoot))
                .setLinearHeadingInterpolation(intakeTunnelEnd.getHeading(), shoot.getHeading())
                .setBrakingStrength(0.1)
                .setBrakingStart(2)
                .build();
    }

    public void nextPath(PathChain path, int pState) {
        if (robot.getFollower().getCurrentTValue() > endPathTValue && robot.getFollower().getCurrentPathNumber() + 1 >= robot.getFollower().getCurrentPathChain().size()) {
            robot.getFollower().followPath(path);
            setPathState(pState, true);
        }
    }

    @Override
    protected JetfireRobot instantiateRobot() {
        return new JetfireRobot();
    }

    @Override
    protected Pose instantiateStartPose() {
        return start;
    }

    @Override
    protected AllianceColor instantiateAllianceColor() {
        return AllianceColor.BLUE;
    }
}
