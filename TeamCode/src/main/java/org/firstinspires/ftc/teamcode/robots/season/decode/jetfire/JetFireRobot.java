package org.firstinspires.ftc.teamcode.robots.season.decode.jetfire;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.hardware.controllers.motor.AngleMotorController;
import org.firstinspires.ftc.teamcode.hardware.controllers.motor.VelocityMotorController;
import org.firstinspires.ftc.teamcode.hardware.controllers.servo.AngleServoController;
import org.firstinspires.ftc.teamcode.hardware.controllers.servo.ServoTimerController;
import org.firstinspires.ftc.teamcode.hardware.controllers.subsystems.Turret;
import org.firstinspires.ftc.teamcode.robots.base.RobotBase;
import org.firstinspires.ftc.teamcode.robots.base.opmodes.OpModeBase;
import org.firstinspires.ftc.teamcode.util.LinearInterpolator;
import org.firstinspires.ftc.teamcode.util.LogicUtil;
import org.firstinspires.ftc.teamcode.util.math.Angle;

import java.util.Optional;
import java.util.TreeMap;

@Configurable
public class JetFireRobot extends RobotBase {
    private Turret turret;
    private static boolean isAutoAimOn = false;

    enum FlywheelMode {
        AUTO,
        SET_VELOCITY,
        OFF
    }
    private static FlywheelMode flywheelMode = FlywheelMode.OFF;
    private static double flywheelSetVelocity = 3000;

    public enum IntakeMode {
        INTAKE,
        REVERSE,
        OFF
    }
    IntakeMode intakeMode = IntakeMode.OFF;

    DcMotorEx intakeMotor;
    private static final double INTAKE_POWER = 435;

    public boolean isAutoHoodAngleOn = true;
    public static double hoodSetAngleDeg = 30;

    public static Pose targetGoal;
    private Angle headingTowardsGoal;
    private double distanceFromGoal;
    private Angle relativeTurntableHeading;
    private Angle absoluteTurntableHeading;
    private boolean isReadyToShoot;

    private LinearInterpolator flywheelVelocityByDistanceInterpolator;
    private LinearInterpolator hoodAngleByDistanceInterpolator;

    ServoTimerController launchServoController;
    private static final double LAUNCH_SERVO_UP = 0.7;
    private static final double LAUNCH_SERVO_DOWN = 0.27;
    // TODO: TEST ANGLE CLASS PANELS

    public static PIDFCoefficients flywheelPIDFCoefficients = new PIDFCoefficients(0.0027, 0, 0, 0.0002);
    public static PIDFCoefficients turntablePIDFCoefficients = new PIDFCoefficients(0.015, 0, 0, 0);

    public static double flywheelVelocityMargin = 60;
    public static double turntableHeadingMarginDeg = 2;

    @Override
    public void init(HardwareMap hardwareMap, Pose startPose, OpModeBase.AllianceColor allianceColor) {
        super.init(hardwareMap, startPose, allianceColor);
        targetGoal = new Pose(-72, allianceColor == OpModeBase.AllianceColor.RED ? 72 : -72);
    }

    @Override
    public void initHardware(HardwareMap hardwareMap) {
        Angle turretStartHeading = Optional.ofNullable(JetFireStaticData.lastTurretHeading)
                    .orElse(new Angle(0));

        turret = new Turret(
                new VelocityMotorController(
                        hardwareMap.get(DcMotorEx.class, "flywheel"),
                        DcMotorSimple.Direction.REVERSE,
                        flywheelPIDFCoefficients,
                        28,
                        1
                ),
                new AngleMotorController(
                        hardwareMap.get(DcMotorEx.class, "turntable"),
                        DcMotorSimple.Direction.FORWARD,
                        turntablePIDFCoefficients,
                        384.5,
                        (double) 64 / 16,
                        new Angle(130, false),
                        new Angle(-130, false),
                        turretStartHeading,
                        true
                ),
                new AngleServoController(
                        hardwareMap.get(Servo.class, "hood"),
                        new Angle(300, false),
                        (double) 48 / 40,
                        new Angle(22, false),
                        new Angle(22 + 19, false),
                        new Angle(22)
                )
        );

        intakeMotor = hardwareMap.get(DcMotorEx.class, "intake");
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        TreeMap<Double, Double> flywheelSpeedByDistanceMap = new TreeMap<>();
        // INCH, RPM
        flywheelSpeedByDistanceMap.put(60.0, 3000.0);
        flywheelVelocityByDistanceInterpolator = new LinearInterpolator(flywheelSpeedByDistanceMap);

        TreeMap<Double, Double> hoodAngleByDistanceMap = new TreeMap<>();
        // INCH, DEGREES
        hoodAngleByDistanceMap.put(60.0, 18.5);
        hoodAngleByDistanceInterpolator = new LinearInterpolator(hoodAngleByDistanceMap);
    }

    @Override
    public void hardwareTelemetry(TelemetryManager telemetry) {
        super.hardwareTelemetry(telemetry);
        telemetry.addLine("");
        telemetry.addLine("- TURNTABLE -");
        telemetry.addData("Heading Towards Goal", headingTowardsGoal.getAngle(Angle.AngleUnit.DEGREES, Angle.AngleSystem.SIGNED_180_WRAPPED));
        telemetry.addData("Turret Heading Error", headingTowardsGoal.getAngle(Angle.AngleUnit.DEGREES, Angle.AngleSystem.SIGNED_180_WRAPPED) - absoluteTurntableHeading.getAngle(Angle.AngleUnit.DEGREES, Angle.AngleSystem.SIGNED_180_WRAPPED));
        telemetry.addData("Distance From Goal", distanceFromGoal);
        telemetry.addData("Relative Turntable Heading", relativeTurntableHeading.getAngle(Angle.AngleUnit.DEGREES, Angle.AngleSystem.SIGNED_180_WRAPPED));
        telemetry.addData("Absolute Turntable Heading", absoluteTurntableHeading.getAngle(Angle.AngleUnit.DEGREES, Angle.AngleSystem.SIGNED_180_WRAPPED));
        telemetry.addData("Target Relative Heading", turret.turntableController().getTargetHeading().getAngle(Angle.AngleUnit.DEGREES, Angle.AngleSystem.SIGNED_180_WRAPPED));
        telemetry.addLine("");
        telemetry.addLine("- FLYWHEEL -");
        telemetry.addData("Flywheel Velocity (RPM)", turret.flywheelController().getVelocity());
        telemetry.addData("Flywheel Target Velocity (RPM)", turret.flywheelController().getTargetVelocity());
        telemetry.addLine("");
        telemetry.addLine("- HOOD -");
        telemetry.addData("Hood Angle", turret.hoodServoController().getTargetAngle().getAngle(Angle.AngleUnit.DEGREES, Angle.AngleSystem.SIGNED));
    }

    @Override
    public void startConfiguration() {

    }

    @Override
    public void updateHardwareStates() {
        Pose displacedPose = targetGoal.minus(follower.getPose());
        headingTowardsGoal = new Angle(Math.atan2(displacedPose.getY(), displacedPose.getX()));
        distanceFromGoal = follower.getPose().distanceFrom(targetGoal);
        relativeTurntableHeading = turret.turntableController().getAngle();
        absoluteTurntableHeading = new Angle(relativeTurntableHeading.getAngle(Angle.AngleSystem.SIGNED_180_WRAPPED) + follower.getHeading());


        Angle targetTurntableHeading = isAutoAimOn ? new Angle(headingTowardsGoal.getAngle(Angle.AngleSystem.SIGNED_180_WRAPPED) - follower.getHeading()) : new Angle(0);
        double flywheelSpeed = switch (flywheelMode) {
            case AUTO -> flywheelVelocityByDistanceInterpolator.interpolate(distanceFromGoal);
            case SET_VELOCITY -> flywheelSetVelocity;
            case OFF -> 0.0;
        };
        Angle hoodAngle = new Angle(isAutoHoodAngleOn ? hoodAngleByDistanceInterpolator.interpolate(distanceFromGoal) : hoodSetAngleDeg, false);


        turret.update(
                flywheelSpeed,
                targetTurntableHeading,
                hoodAngle
        );

        // INTAKE
        intakeMotor.setPower(switch (intakeMode) {
            case INTAKE -> INTAKE_POWER;
            case REVERSE -> -INTAKE_POWER;
            case OFF -> 0;
        });


        JetFireStaticData.lastTurretHeading = turret.turntableController().getAngle();


//        launchServoController.update();

        // Testing
        if (flywheelPIDFCoefficients != turret.flywheelController().getPidfController().getCoefficients()) turret.flywheelController().getPidfController().setCoefficients(flywheelPIDFCoefficients);
        if (turntablePIDFCoefficients != turret.turntableController().getPidfController().getCoefficients()) turret.turntableController().getPidfController().setCoefficients(turntablePIDFCoefficients);

        super.updateHardwareStates();
    }

    public void shoot() {
        // wait for reset
        launchServoController.setPosition(LAUNCH_SERVO_UP, 200, LAUNCH_SERVO_DOWN, 200);
    }

    public void setIntakeMode(IntakeMode intakeMode) {
        this.intakeMode = intakeMode;
    }

    public boolean isAutoAimOn() {
        return isAutoAimOn;
    }

    public void setAutoAimOn(boolean autoAimOn) {
        isAutoAimOn = autoAimOn;
    }

    public boolean isReadyToShoot() {
        boolean isFlywheelReady = LogicUtil.isWithinRange(
                turret.flywheelController().getVelocity(),
                turret.flywheelController().getTargetVelocity(),
                flywheelVelocityMargin
        );

        boolean isTurntableReady = LogicUtil.isWithinRange(
                turret.turntableController().getAngle().getAngle(Angle.AngleUnit.DEGREES, Angle.AngleSystem.SIGNED_180_WRAPPED),
                turret.turntableController().getTargetHeading().getAngle(Angle.AngleUnit.DEGREES, Angle.AngleSystem.SIGNED_180_WRAPPED),
                turntableHeadingMarginDeg
        );

        return isFlywheelReady && isTurntableReady;
    }

    @Override
    protected FieldType instantiateFieldType() {
        return FieldType.SQUARE_INVERTED_ALLIANCE;
    }

    @Override
    public Follower instantiateFollower(HardwareMap hardwareMap) {
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
                        .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
                        .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
                        .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
                        .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
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

    @Override
    public Limelight3A instantiateLimelight3A(HardwareMap hardwareMap) {
        return null;
    }
}
