package org.firstinspires.ftc.teamcode.robots.base;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.robots.base.opmodes.OpModeBase;

public class StaticData {
    public static Pose lastPose = new Pose(0 , 0, 0);
    public static OpModeBase.AllianceColor allianceColor = OpModeBase.AllianceColor.BLUE;
}
