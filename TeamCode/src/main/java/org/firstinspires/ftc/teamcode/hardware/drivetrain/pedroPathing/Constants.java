package org.firstinspires.ftc.teamcode.hardware.drivetrain.pedroPathing;

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.robots.season.decode.jetfire.JetFireRobot;

public class Constants {
//    public static FollowerConstants followerConstants = new FollowerConstants()
//            .mass(7)
//            .forwardZeroPowerAcceleration(-32)
//            .lateralZeroPowerAcceleration(-54.1)
//            .translationalPIDFCoefficients(new PIDFCoefficients(0.1, 0, 0.01, 0))
//            .headingPIDFCoefficients(new PIDFCoefficients(1, 0, 0.012, 0.01));

//    public static MecanumConstants driveConstants = new MecanumConstants()
//            .maxPower(1)
//            .rightFrontMotorName("front-right")
//            .rightRearMotorName("rear-right")
//            .leftRearMotorName("rear-left")
//            .leftFrontMotorName("front-left")
//            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
//            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
//            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
//            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
//            .xVelocity(87.1)
//            .yVelocity(75.3);

//    public static PinpointConstants localizerConstants = new PinpointConstants()
//            .forwardPodY(0)
//            .strafePodX(0)
//            .distanceUnit(DistanceUnit.INCH)
//            .hardwareMapName("odometry")
//            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
//            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
//            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);


//    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return createJetFireFollower(hardwareMap);
//        return new FollowerBuilder(new FollowerConstants()
//                .mass(7)
//                .forwardZeroPowerAcceleration(-32)
//                .lateralZeroPowerAcceleration(-54.1)
//                .translationalPIDFCoefficients(new PIDFCoefficients(0.1, 0, 0.01, 0))
//                .headingPIDFCoefficients(new PIDFCoefficients(1, 0, 0.012, 0.01)), hardwareMap)
//                .pathConstraints(new PathConstraints(0.99, 100, 1, 1))
//                .mecanumDrivetrain(new MecanumConstants()
//                        .maxPower(1)
//                        .rightFrontMotorName("front-right")
//                        .rightRearMotorName("rear-right")
//                        .leftRearMotorName("rear-left")
//                        .leftFrontMotorName("front-left")
//                        .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
//                        .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
//                        .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
//                        .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
//                        .xVelocity(87.1)
//                        .yVelocity(75.3))
//                .pinpointLocalizer(new PinpointConstants()
//                        .forwardPodY(0)
//                        .strafePodX(0)
//                        .distanceUnit(DistanceUnit.INCH)
//                        .hardwareMapName("odometry")
//                        .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
//                        .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
//                        .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD))
//                .build();
    }

    public static Follower createDecodeFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(new FollowerConstants()
                .mass(11)
                .forwardZeroPowerAcceleration(-28.2)
                .lateralZeroPowerAcceleration(-67.5)
                .translationalPIDFCoefficients(new PIDFCoefficients(0.1, 0, 0.01, 0))
                .headingPIDFCoefficients(new PIDFCoefficients(
                        1.05,
                        0,
                        0.01,
                        0.01)), hardwareMap)
                .pathConstraints(new PathConstraints(0.99, 100, 1, 1))
                .mecanumDrivetrain(new MecanumConstants()
                        .maxPower(1)
                        .rightFrontMotorName("front-right")
                        .rightRearMotorName("rear-right")
                        .leftRearMotorName("rear-left")
                        .leftFrontMotorName("front-left")
                        .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
                        .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
                        .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
                        .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
                        .xVelocity(79.5)
                        .yVelocity(64.5))
                .pinpointLocalizer(new PinpointConstants()
                        .forwardPodY(24.75)
                        .strafePodX(142.8)
                        .distanceUnit(DistanceUnit.MM)
                        .hardwareMapName("odometry")
                        .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
                        .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
                        .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED))
                .build();
    }

    public static Follower createJetFireFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(new FollowerConstants()
                .mass(19)
                .forwardZeroPowerAcceleration(-28.2)
                .lateralZeroPowerAcceleration(-67.5)
                .translationalPIDFCoefficients(new PIDFCoefficients(0.1, 0, 0.01, 0))
                .headingPIDFCoefficients(new PIDFCoefficients(
                        1.05,
                        0,
                        0.01,
                        0.01)), hardwareMap)
                .pathConstraints(new PathConstraints(0.99, 100, 1, 1))
                .mecanumDrivetrain(new MecanumConstants()
                        .maxPower(1)
                        .rightFrontMotorName("front-right")
                        .rightRearMotorName("rear-right")
                        .leftRearMotorName("rear-left")
                        .leftFrontMotorName("front-left")
                        .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE) // p. 2 control
                        .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE) // p. 3 control
                        .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD) // p.1 expansion
                        .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD) // p.0 expansion
                        .xVelocity(79.5)
                        .yVelocity(64.5))
                .pinpointLocalizer(new PinpointConstants()
                        .forwardPodY(168.5)
                        .strafePodX(-67)
                        .distanceUnit(DistanceUnit.MM)
                        .hardwareMapName("odometry")
                        .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
                        .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
                        .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD))
                .build();
    }
}
