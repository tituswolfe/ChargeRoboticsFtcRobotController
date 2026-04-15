package org.firstinspires.ftc.teamcode.hardware.drivetrain.pedroPathing;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PredictiveBrakingCoefficients;
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

@Configurable
public class Constants {
//    public static double jetfireMassKg = 14.8;
//    public static PIDFCoefficients jetfireTranslationalPIDFCoefficients = new PIDFCoefficients(0.1, 0, 0.01, 0.02);
//    public static PIDFCoefficients jetfireHeadingPIDFCoefficients = new PIDFCoefficients(4, 0, 0.1, 0.02);

    public static PathConstraints JETFIRE_PATH_CONSTRAINTS = new PathConstraints(0.99, 100, 1, 0.9);

    public static MecanumConstants JETFIRE_MECANUM_DRIVETRAIN = new MecanumConstants()
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

    public static PinpointConstants JETFIRE_PINPOINT_CONSTRAINTS = new PinpointConstants()
                        .forwardPodY(168.5)
                        .strafePodX(-67)
                        .distanceUnit(DistanceUnit.MM)
                        .hardwareMapName("odometry")
                        .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
                        .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
                        .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);

    public static FollowerConstants JETFIRE_FOLLOWER_CONSTANTS = new FollowerConstants()
            .headingPIDFCoefficients(new PIDFCoefficients(1.5, 0, 0.1, 0)) // tuned constants
            .predictiveBrakingCoefficients(new PredictiveBrakingCoefficients(0.1, 0.06382, 0.0017577)) // (kP, kLinear, kQuadratic)
            .centripetalScaling(0);


//     new FollowerConstants()
//                        .mass(jetfireMassKg)
//                        .forwardZeroPowerAcceleration(-35.92)
//                        .lateralZeroPowerAcceleration(-64.54)
//                        .translationalPIDFCoefficients(jetfireTranslationalPIDFCoefficients)
//                        .headingPIDFCoefficients(jetfireHeadingPIDFCoefficients),
//    hardwareMap)

    public static Follower createJetfireFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(JETFIRE_FOLLOWER_CONSTANTS, hardwareMap)
                .pathConstraints(JETFIRE_PATH_CONSTRAINTS)
                .mecanumDrivetrain(JETFIRE_MECANUM_DRIVETRAIN)
                .pinpointLocalizer(JETFIRE_PINPOINT_CONSTRAINTS)
                .build();
    }

    public static Follower createFollower(HardwareMap hardwareMap) {
        return createJetfireFollower(hardwareMap);
    }
}
