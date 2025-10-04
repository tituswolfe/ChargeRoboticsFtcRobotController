package org.firstinspires.ftc.teamcode.robots.drivechassis;

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.hardware.drivetrain.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.robots.base.RobotBase;

public class DriveRobot extends RobotBase {
    @Override
    public void initHardware(HardwareMap hardwareMap) {

    }

    @Override
    public Follower instantiateFollower(HardwareMap hardwareMap) {
        return Constants.createFollower(hardwareMap);
    }

    @Override
    public Limelight3A instantiateLimelight3A(HardwareMap hardwareMap) {
        return null;
    }
}
