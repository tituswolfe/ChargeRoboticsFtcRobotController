package org.firstinspires.ftc.teamcode.util.actionsequence;

import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;

public class PathAction extends Action {
    private final Follower follower;
    public static double END_OF_PATH_T_VALUE = 0.97;

    public PathAction(PathChain pathChain, Follower follower) {
        super(() -> follower.followPath(pathChain));
        this.follower = follower;
    }

    @Override
    public boolean isFinished() {
        boolean endOfCurrentPath = follower.getCurrentTValue() > END_OF_PATH_T_VALUE;
        boolean isLastPath = follower.getCurrentPathNumber() + 1 >= follower.getCurrentPathChain().size();
        return endOfCurrentPath && isLastPath;
    }
}
