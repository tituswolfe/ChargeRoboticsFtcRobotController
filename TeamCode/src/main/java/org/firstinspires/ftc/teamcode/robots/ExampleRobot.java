package org.firstinspires.ftc.teamcode.robots;

import static org.firstinspires.ftc.teamcode.util.ThreadUtil.sleep;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.hardware.drivetrain.DriveTrain;
import org.firstinspires.ftc.teamcode.hardware.extensions.LinearSlide;
import org.firstinspires.ftc.teamcode.hardware.odometry.GoBildaOdometry;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.hardware.odometry.GoBildaPinpointDriver;

@Config
public class ExampleRobot extends RobotBase {
    // Hardware & variables
    private DriveTrain driveTrain;
    private GoBildaOdometry odometry;
    LinearSlide linearSlide;

    public static PIDCoefficients driveCoefficients = new PIDCoefficients(0.1, 0, 0);
    public static PIDCoefficients turnCoefficients =new PIDCoefficients(0.025, 0, 0);


    // private IMU imu;

    // Setup

    public ExampleRobot(String name) {
        super(name);
    }

    @Override
    public boolean initHardware(HardwareMap hardwareMap) {
        odometry = new GoBildaOdometry(hardwareMap.get(GoBildaPinpointDriver.class,"odometry"), GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);

        // TODO: vision & april tags

//        imu = hardwareMap.get(IMU.class, "imu");
//        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
//                RevHubOrientationOnRobot.LogoFacingDirection.UP,
//                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT
//        )));

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

//        linearSlide = new LinearSlide(
//                hardwareMap.get(DcMotorEx.class, "four_bar_lift"),
//                DcMotorSimple.Direction.REVERSE,
//                9
//        );
//        linearSlide.reset(1000, 20);

        return true;
    }

    // Functions

    public void extendSlide() {
        linearSlide.setPosition(3);
        sleep(2000);
        linearSlide.setPosition(5);
        sleep(2000);
        linearSlide.setPosition(1);
    }

    public void testManuver() {
//        driveTrain.driveTo(0, 24, 0, 0.5);
//        driveTrain.driveTo(-4, 24, 0, 0.5);
//        driveTrain.driveTo(0, 24, 0, 0.5);
//        sleep(2000);
        driveTrain.splineTo(new Pose2D(DistanceUnit.INCH, 0, 24, AngleUnit.DEGREES, 0));

    }

    // Getters & Setters

    // Note: For IntelliJ/Android Studio, use  ALT + INSERT to add getters quickly

    public DriveTrain getDriveTrain() {
        return driveTrain;
    }

    public GoBildaOdometry getOdometry() {
        return odometry;
    }

    public LinearSlide getLinearSlide() {
        return linearSlide;
    }
}
