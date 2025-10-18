package org.firstinspires.ftc.teamcode.robots.base;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.robots.base.opmodes.BaseOpMode;

public class StaticData {
    public static Pose lastAutoPose = new Pose(0, 0, 0);

    public static BaseOpMode.AllianceColor allianceColor;
    public static RobotBase.FieldType lastFieldType;
}
