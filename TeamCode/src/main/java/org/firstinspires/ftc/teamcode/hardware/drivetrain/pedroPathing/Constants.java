package org.firstinspires.ftc.teamcode.hardware.drivetrain.pedroPathing;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Configurable
public class Constants {
    public static PIDFCoefficients jetfireTranslationalPIDFCoefficients = new PIDFCoefficients(0.1, 0, 0.01, 0.02);
    public static PIDFCoefficients jetfireHeadingPIDFCoefficients = new PIDFCoefficients(4, 0, 0.1, 0.02);

    public static PathConstraints jetfirePathConstraints = new PathConstraints(0.99, 100, 1, 0.9);

    public static MecanumConstants jetfireMecanumConstraints = new MecanumConstants()
                        .maxPower(1)
                        .rightFrontMotorName("front-right")
                        .rightRearMotorName("rear-right")
                        .leftRearMotorName("rear-left")
                        .leftFrontMotorName("front-left")
                        .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE) // p. 2 control
                        .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE) // p. 3 control
                        .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD) // p.1 expansion
                        .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD) // p.0 expansion
                        .xVelocity(64.67)
                        .yVelocity(51.9);

    public static PinpointConstants jetfirePinpointConstraints = new PinpointConstants()
                        .forwardPodY(168.5)
                        .strafePodX(-67)
                        .distanceUnit(DistanceUnit.MM)
                        .hardwareMapName("odometry")
                        .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
                        .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
                        .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return createJetfireFollower(hardwareMap);
    }

    public static Follower createDecodeFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(new FollowerConstants()
                .mass(11)
                .forwardZeroPowerAcceleration(-28.2)
                .lateralZeroPowerAcceleration(-67.5)
                .translationalPIDFCoefficients(new PIDFCoefficients(0.1, 0, 0.01, 0))
                .headingPIDFCoefficients(
                        new PIDFCoefficients(
                        1.05,
                        0,
                        0.01,
                        0.01)
                ), hardwareMap
        )
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

    public static Follower createJetfireFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(
                new FollowerConstants()
                        .mass(14.6)
                        .forwardZeroPowerAcceleration(-35.92)
                        .lateralZeroPowerAcceleration(-64.54)
                        .translationalPIDFCoefficients(jetfireTranslationalPIDFCoefficients)
                        .headingPIDFCoefficients(jetfireHeadingPIDFCoefficients),
                hardwareMap)
                .pathConstraints(jetfirePathConstraints)
                .mecanumDrivetrain(jetfireMecanumConstraints)
                .pinpointLocalizer(jetfirePinpointConstraints)
                .build();
    }
}
