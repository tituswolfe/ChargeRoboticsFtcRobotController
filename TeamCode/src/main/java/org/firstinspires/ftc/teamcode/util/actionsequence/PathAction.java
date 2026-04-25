package org.firstinspires.ftc.teamcode.util.actionsequence;

import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.robot.Robot;

import org.firstinspires.ftc.teamcode.robots.season.decode.jetfire.JetfireRobot;

public class PathAction extends Action {
    private final Follower follower;
    public static double END_OF_PATH_T_VALUE = 0.97;

    public PathAction(PathChain pathChain, JetfireRobot robot) {
        super(() -> robot.getFollower().followPath(pathChain));
        this.follower = robot.getFollower();
    }

    @Override
    public boolean isFinished() {
        boolean endOfCurrentPath = follower.getCurrentTValue() > END_OF_PATH_T_VALUE;
        boolean isLastPath = follower.getCurrentPathNumber() + 1 >= follower.getCurrentPathChain().size();
        return endOfCurrentPath && isLastPath;
    }
}
