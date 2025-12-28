package org.firstinspires.ftc.teamcode.robots.season.decode.jetfire;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.hardware.controllers.motor.TurntableMotorController;
import org.firstinspires.ftc.teamcode.hardware.controllers.motor.VelocityMotorController;
import org.firstinspires.ftc.teamcode.hardware.controllers.servo.AngleServoController;
import org.firstinspires.ftc.teamcode.hardware.controllers.servo.RGBIndicatorLightController;
import org.firstinspires.ftc.teamcode.hardware.controllers.servo.ServoTimerController;
import org.firstinspires.ftc.teamcode.hardware.controllers.subsystems.Turret;
import org.firstinspires.ftc.teamcode.hardware.drivetrain.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.robots.base.RobotBase;
import org.firstinspires.ftc.teamcode.robots.base.opmodes.OpModeBase;
import org.firstinspires.ftc.teamcode.util.LinearInterpolator;
import org.firstinspires.ftc.teamcode.util.info.HardwareInfo;
import org.firstinspires.ftc.teamcode.util.math.Angle;
import org.firstinspires.ftc.teamcode.util.math.MathUtil;
import org.firstinspires.ftc.teamcode.util.math.PhysicsUtil;

import java.util.List;
import java.util.Optional;
import java.util.TreeMap;

@Configurable
public class JetfireRobot extends RobotBase {
    private Turret turret;

    private boolean isAutoAimOn = false;

    // FLYWHEELS
    public enum FlywheelMode {
        AUTO,
        SET_VELOCITY,
        OFF
    }
    private FlywheelMode flywheelMode = FlywheelMode.OFF;
    public static double flywheelSetVelocity = 3000;
    public static PIDFCoefficients flywheelPIDFCoefficients = new PIDFCoefficients(0.0027, 0, 0, 0.0002);

    // TURNTABLE
    public static PIDFCoefficients turntablePIDFCoefficients = new PIDFCoefficients(0.015 / 2, 0, 0, 0);

    // HOOD
    public static boolean isAutoHoodAngleOn = false;
    public static double hoodSetAngleDeg = 22;

    // INTAKE
    public enum IntakeMode {
        INTAKE,
        REVERSE,
        OFF
    }
    IntakeMode intakeMode = IntakeMode.OFF;
    DcMotorEx intakeMotor;
    private static final double INTAKE_POWER = 0.6;

    private static Pose resetPose; // HUMAN PLAYER ZONE

    // TODO: Better way? rather than 3 different interpolators?
    private LinearInterpolator flywheelVelocityByDistanceInterpolator;
    private LinearInterpolator hoodAngleByDistanceInterpolator;
    private LinearInterpolator timeOfFlightByDistanceInterpolator;

    // TRANSFER SERVO
    ServoTimerController transferServoController;
    public static double TRANSFER_SERVO_UP = 0.7;
    public static double TRANSFER_SERVO_DOWN = 0.27;

    private static final long LAUNCH_SERVO_DELAY_MS = 300;
    // TODO: TEST ANGLE CLASS PANELS (WOULD HAVE TO BE PUBLIC FOUND OUT)
// TODO: PEDRO PATHING RIGHT COORDINATES?

    // GATE SERVO
    Servo gateServo;
    public static double GATE_SERVO_OPEN = 0.1;
    public static double GATE_SERVO_CLOSED = 0.0;
    // TODO: Naming convent for static public vs final?

    // ARTIFACT CHAMBER DETECTION
    DistanceSensor distanceSensor;
    public static double ARTIFACT_DETECTION_THRESHOLD_MM = 0;

    Timer laucnhCooldownTimer = new Timer();
    public static double LAUNCH_COOLDOWN_MS = 100 + LAUNCH_SERVO_DELAY_MS; // TODO?

    RGBIndicatorLightController indicatorLightController;

    // MARGINS
    public static double flywheelVelocityMargin = 60;
    public static double turntableHeadingMarginDeg = 2;


    // PUBLIC STATIC VARIABLES
    public static Pose targetGoal;
    public static boolean testing = false;

    private boolean isReadyToShoot = false;


    @Override
    public void init(HardwareMap hardwareMap, Pose startPose, OpModeBase.AllianceColor allianceColor) {
        super.init(hardwareMap, startPose, allianceColor);
        targetGoal = new Pose(-72, allianceColor == OpModeBase.AllianceColor.RED ? 72 : -72);
        // targetAim
    }

    @Override
    public void initHardware(HardwareMap hardwareMap) {
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        Angle turretStartHeading = Optional.ofNullable(JetfireStaticData.lastTurretHeading)
                    .orElse(new Angle(0));

        VelocityMotorController flywheelController = new VelocityMotorController(
                hardwareMap.get(DcMotorEx.class, "flywheel"),
                DcMotorSimple.Direction.REVERSE,
                flywheelPIDFCoefficients,
                HardwareInfo.GOBILDA_5203_YELLOW_JACKET_MOTOR_6000_RPM.ENCODER_RESOLUTION_PPR,
                1,
                1
        );

        TurntableMotorController turntableController = new TurntableMotorController(
                hardwareMap.get(DcMotorEx.class, "turntable"),
                DcMotorSimple.Direction.FORWARD,
                turntablePIDFCoefficients,
                HardwareInfo.GOBILDA_5203_YELLOW_JACKET_MOTOR_435_RPM.ENCODER_RESOLUTION_PPR, //384.5,
                64.0 / 16.0,
                0.7,
                new Angle(130, false),
                new Angle(-130, false),
                turretStartHeading,
                true
        );

        AngleServoController hoodController = new AngleServoController(
                hardwareMap.get(Servo.class, "hood"),
                Servo.Direction.REVERSE,
                new Angle(300, false),
                (48.0 / 40.0) * (116.0 / 12.0),
                new Angle(16, false),
                new Angle(36, false),
                new Angle(16, false)
        );

        turret = new Turret(
                flywheelController,
                turntableController,
                hoodController
        );

        intakeMotor = hardwareMap.get(DcMotorEx.class, "intake");
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        indicatorLightController = new RGBIndicatorLightController(hardwareMap.get(Servo.class, "indicator"));

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
    public void startConfiguration() {

    }

    @Override
    public void update(long deltaTimeMs, TelemetryManager telemetry) {
        super.update(deltaTimeMs, telemetry);

        Pose displacedPose = targetGoal.minus(follower.getPose());
        // TODO: AIM GOAL POINT?
        Angle goalHeading = new Angle(Math.atan2(displacedPose.getY(), displacedPose.getX()));
        double distanceFromGoal = follower.getPose().distanceFrom(targetGoal);
        Angle relativeTurntableHeading = turret.turntableController().getHeading();
        Angle absoluteTurntableHeading = new Angle(relativeTurntableHeading.getAngle(Angle.AngleSystem.SIGNED_180_WRAPPED) + follower.getHeading());


        boolean isFlywheelReady = MathUtil.isWithinRange(
                turret.flywheelController().getVelocity(),
                turret.flywheelController().getTargetVelocity(),
                flywheelVelocityMargin
        ) && !flywheelMode.equals(FlywheelMode.OFF);

        boolean isTurntableReady = MathUtil.isWithinRange(
                turret.turntableController().getHeading().getAngle(Angle.AngleUnit.DEGREES, Angle.AngleSystem.SIGNED_180_WRAPPED),
                turret.turntableController().getTargetHeading().getAngle(Angle.AngleUnit.DEGREES, Angle.AngleSystem.SIGNED_180_WRAPPED),
                turntableHeadingMarginDeg
        );

        boolean isArtifactLoaded = distanceSensor.getDistance(DistanceUnit.MM) < ARTIFACT_DETECTION_THRESHOLD_MM;

        boolean isCooldownOver = laucnhCooldownTimer.getElapsedTime() > LAUNCH_COOLDOWN_MS;

        isReadyToShoot = isFlywheelReady && isTurntableReady && isArtifactLoaded && isCooldownOver;


        long launchDelay = deltaTimeMs + LAUNCH_SERVO_DELAY_MS;

        Pose currentPose = follower.getPose();
        Vector velocity = follower.getVelocity();
        double angularVelocity = follower.getAngularVelocity();
        Vector acceleration = follower.getAcceleration();
        double angularAcceleration = 0;

        // TODO: Boolean for testing futures

        // TODO ACTIVE SHOOTING MECHANISM CHECKS
        // TODO KNOW WHEN YOU SHOOT TELEOP AND AUTO, DON"T JUST HOLD THE GATE OPEN
        // Lead Computing
        Pose futurePose = PhysicsUtil.predictFuturePose(
                currentPose,
                velocity,
                angularVelocity,
                acceleration,
                angularAcceleration,
                (launchDelay / 1000.0)
        );


        double distanceToGoal = futurePose.distanceFrom(targetGoal);
        double timeOfFlightMs = timeOfFlightByDistanceInterpolator.interpolate(distanceToGoal);

        Pose virtualGoal = PhysicsUtil.predictFuturePose(
                targetGoal,
                velocity.times(-1),
                0,
                new Vector(0, 0),
                0,
                (timeOfFlightMs / 1000.0)
        );

        double virtualDistanceFromGoal = futurePose.distanceFrom(virtualGoal);
        Angle virtualGoalHeading = new Angle(Math.atan2(displacedPose.getY(), displacedPose.getX()));


        // TODO: Deceleration coefficent due to air resitance on the virutal goal?
        // TODO: LUT for offset as you get farther away?
        // TODO: Feedforward on turntable

        double flywheelSpeed = switch (flywheelMode) {
            case AUTO -> flywheelVelocityByDistanceInterpolator.interpolate(virtualDistanceFromGoal);
            case SET_VELOCITY -> flywheelSetVelocity;
            case OFF -> 0.0;
        };
        Angle targetTurntableHeading = isAutoAimOn ? new Angle(virtualGoalHeading.getAngle(Angle.AngleSystem.SIGNED_180_WRAPPED) - follower.getHeading()) : new Angle(0);
        Angle hoodAngle = new Angle(isAutoHoodAngleOn ? hoodAngleByDistanceInterpolator.interpolate(virtualDistanceFromGoal) : hoodSetAngleDeg, false);

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


        JetfireStaticData.lastTurretHeading = turret.turntableController().getHeading();

        indicatorLightController.setColor(isReadyToShoot() ? RGBIndicatorLightController.Color.GREEN : RGBIndicatorLightController.Color.RED);

        transferServoController.update();

        if (testing) {
            if (!turret.flywheelController().getPidfController().getCoefficients().equals(flywheelPIDFCoefficients)) turret.flywheelController().getPidfController().setCoefficients(flywheelPIDFCoefficients);
            if (!turret.turntableController().getPidfController().getCoefficients().equals(turntablePIDFCoefficients)) turret.turntableController().getPidfController().setCoefficients(turntablePIDFCoefficients);

        }


        telemetry.addLine("");
        telemetry.addLine("- VARIABLES -");
        telemetry.addData("GOAL DISTANCE", distanceFromGoal);
        telemetry.addData("GOAL HEADING", goalHeading.getAngle(Angle.AngleUnit.DEGREES, Angle.AngleSystem.SIGNED_180_WRAPPED));
        telemetry.addLine("");
        telemetry.addLine("- TURNTABLE -");
        telemetry.addData("ABSOLUTE TURNTABLE ERROR", goalHeading.getAngle(Angle.AngleUnit.DEGREES, Angle.AngleSystem.SIGNED_180_WRAPPED) - absoluteTurntableHeading.getAngle(Angle.AngleUnit.DEGREES, Angle.AngleSystem.SIGNED_180_WRAPPED));
        telemetry.addData("RELATIVE TURNTABLE HEADING", relativeTurntableHeading.getAngle(Angle.AngleUnit.DEGREES, Angle.AngleSystem.SIGNED_180_WRAPPED));
        telemetry.addData("ABSOLUTE TURNTABLE HEADING", absoluteTurntableHeading.getAngle(Angle.AngleUnit.DEGREES, Angle.AngleSystem.SIGNED_180_WRAPPED));
        telemetry.addData("RELATIVE GOAL TURNTABLE HEADING", turret.turntableController().getTargetHeading().getAngle(Angle.AngleUnit.DEGREES, Angle.AngleSystem.SIGNED_180_WRAPPED));
        telemetry.addLine("");
        telemetry.addLine("- FLYWHEEL -");
        telemetry.addData("FLYWHEEL VELOCITY RPM", turret.flywheelController().getVelocity());
        telemetry.addData("FLYWHEEL TARGET VELOCITY RPM", turret.flywheelController().getTargetVelocity());
        telemetry.addLine("");
        telemetry.addLine("- HOOD -");
        telemetry.addData("HOOD ANGLE", turret.hoodServoController().getTargetAngle().getAngle(Angle.AngleUnit.DEGREES, Angle.AngleSystem.SIGNED));
    }

    public void shoot() {
        transferServoController.setPosition(TRANSFER_SERVO_UP, 100, TRANSFER_SERVO_DOWN);
        laucnhCooldownTimer.resetTimer();
    }

    // TODO: GATE SERVO
    // TODO OPEN ON SHOOT, OR OPEN ENTIRLY WHILE TRANSFERING ?

    public void setIntakeMode(IntakeMode intakeMode) {
        this.intakeMode = intakeMode;
    }

    public IntakeMode getIntakeMode() {
        return intakeMode;
    }

    public boolean isAutoAimOn() {
        return isAutoAimOn;
    }

    public void setAutoAimOn(boolean autoAimOn) {
        isAutoAimOn = autoAimOn;
    }

    public FlywheelMode getFlywheelMode() {
        return flywheelMode;
    }

    public void setFlywheelMode(FlywheelMode flywheelMode) {
        this.flywheelMode = flywheelMode;
    }

    public boolean isReadyToShoot() {
        return isReadyToShoot;

    }

    @Override
    protected FieldType instantiateFieldType() {
        return FieldType.SQUARE_INVERTED_ALLIANCE;
    }

    @Override
    public Follower instantiateFollower(HardwareMap hardwareMap) {
        return Constants.createJetFireFollower(hardwareMap);
    }

    @Override
    public Limelight3A instantiateLimelight3A(HardwareMap hardwareMap) {
        return null;
    }
}
