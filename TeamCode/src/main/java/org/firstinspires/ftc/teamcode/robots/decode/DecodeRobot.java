package org.firstinspires.ftc.teamcode.robots.decode;

import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardware.RPSController;
import org.firstinspires.ftc.teamcode.robots.base.RobotBase;

public class DecodeRobot extends RobotBase {
    enum Position {
        RED_OBELISK,
        BLUE_OBELISK,
        RED_AUDIENCE,
        BLUE_AUDIENCE
    }
    public static final int redGoalAprilTagID = 22;
    public static final int blueGoalAprilTagID = 22;
    public static final int purpleGreenPorpleMotif = 22;

    public RPSController flywheel1;
    public RPSController flywheel2;

    @Override
    public void initHardware(HardwareMap hardwareMap) {
        DcMotorEx flywheelMotor1 = hardwareMap.get(DcMotorEx.class, "top-flywheel");
        flywheelMotor1.setDirection(DcMotorSimple.Direction.FORWARD);

        DcMotorEx flywheelMotor2 = hardwareMap.get(DcMotorEx.class, "bottom-flywheel");
        flywheelMotor2.setDirection(DcMotorSimple.Direction.REVERSE);

        flywheel1 = new RPSController(flywheelMotor1, 28);
        flywheel2 = new RPSController(flywheelMotor2, 28);
    }

    @Override
    public Follower instantiateFollower(HardwareMap hardwareMap) {
        return null; //createFollower(hardwareMap);
    }

    @Override
    public Limelight3A instantiateLimelight3A(HardwareMap hardwareMap) {
        return null; // hardwareMap.get(Limelight3A.class, "limelight");
    }
}
