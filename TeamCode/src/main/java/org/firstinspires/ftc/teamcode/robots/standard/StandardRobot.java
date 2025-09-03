package org.firstinspires.ftc.teamcode.robots.standard;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.hardware.drivetrain.DriveTrain;
import org.firstinspires.ftc.teamcode.hardware.odometry.GoBildaOdometry;
import org.firstinspires.ftc.teamcode.hardware.odometry.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.robots.base.RobotBase;

public class StandardRobot extends RobotBase {
    private DriveTrain driveTrain;
    private GoBildaOdometry odometry;

    public static PIDCoefficients driveCoefficients = new PIDCoefficients(0.1, 0, 0);
    public static PIDCoefficients turnCoefficients = new PIDCoefficients(0.025, 0, 0);

    public StandardRobot(String name) {
        super(name);
    }

    @Override
    public boolean initHardware(HardwareMap hardwareMap) {
        odometry = new GoBildaOdometry(hardwareMap.get(GoBildaPinpointDriver.class,"odometry"), GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);

        driveTrain = new DriveTrain(
                hardwareMap.get(DcMotorEx.class, "front_left"),
                hardwareMap.get(DcMotorEx.class, "front_right"),
                hardwareMap.get(DcMotorEx.class, "back_left"),
                hardwareMap.get(DcMotorEx.class, "back_right"),
                odometry,
                driveCoefficients, // d 0.051
                turnCoefficients,
                DistanceUnit.MM,
                104,
                384.5,
                0.05
        );
        return true;
    }

    // alt + insert to add getters & setters

    public DriveTrain getDriveTrain() {
        return driveTrain;
    }

    public GoBildaOdometry getOdometry() {
        return odometry;
    }
}
