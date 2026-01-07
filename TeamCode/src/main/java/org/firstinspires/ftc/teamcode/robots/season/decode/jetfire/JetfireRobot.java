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
import org.firstinspires.ftc.teamcode.robots.base.StaticData;
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
    // TURRET
    private Turret turret;

    // TURNTABLE
    private boolean autoAimTurntable = false;

    // FLYWHEELS
    public enum FlywheelMode {
        AUTO,
        OFF
    }
    private FlywheelMode flywheelMode = FlywheelMode.OFF;
    public static PIDFCoefficients flywheelPIDFCoefficients = new PIDFCoefficients(0.0027, 0, 0, 0.0002);

    // TURNTABLE
    public static PIDFCoefficients turntablePIDFCoefficients = new PIDFCoefficients(0.045, 0, 0.0012, 0);

    // INTAKE
    DcMotorEx intakeMotor;
    public static double INTAKE_POWER = 1;
    public static double REVERSE_INTAKE_POWER = -0.7;
    public enum IntakeMode {
        ON,
        OFF
    }
    IntakeMode intakeMode = IntakeMode.OFF;
    private boolean reverseIntake = false;

    // LUTs
    private LinearInterpolator flywheelVelocityByDistanceInterpolator;
    private LinearInterpolator hoodAngleByDistanceInterpolator;
    private LinearInterpolator timeOfFlightByDistanceInterpolator;
    private LinearInterpolator turntableOffsetByDistanceInterpolator;

    // TRANSFER SERVO
    ServoTimerController transferServoController;
    public static double TRANSFER_SERVO_UP = 0.4;
    public static double TRANSFER_SERVO_DOWN = 0.67;
    public static int TRANSFER_SERVO_TIME_MS = 200;

    // GATE SERVO
    ServoTimerController gateServoController;
    public static double GATE_SERVO_OPEN = 0.6;
    public static double GATE_SERVO_CLOSED = 0.8;
    public static int GATE_SERVO_TIME_MS = 200;

    // ARTIFACT CHAMBER DETECTION
    DistanceSensor chamberDistanceSensor;
    public static double ARTIFACT_DETECTION_THRESHOLD_MM = 90;

    // COOLDOWN
    Timer laucnhCooldownTimer = new Timer();
    public static int LAUNCH_COOLDOWN_MS = 150;

    // LIGHTS
    RGBIndicatorLightController indicatorLightController;

    // MARGINS
    public static double FLYWHEEL_VELOCITY_MARGIN_RPM = 60;
    public static double TURNTABLE_HEADING_MARGIN_DEG = 2;

    public static long LAUNCH_DELAY_MS = 300;

    // TUNING
    public static boolean TUNING = false;
    public static double FLYWHEEL_TARGET_VELOCITY = 0;
    public static double HOOD_ANGLE = 16;
    public static double TURNTABLE_OFFSET_DEG;

    // RUNTIME
    public static Pose targetGoal;
    private boolean isReadyToShoot = false;
    private double driverTurntableOffset = 0;

    @Override
    public void init(HardwareMap hardwareMap, Pose startPose, OpModeBase.AllianceColor allianceColor) {
        super.init(hardwareMap, startPose, allianceColor);
        targetGoal = new Pose(-72, allianceColor == OpModeBase.AllianceColor.RED ? 72 : -72);
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
                HardwareInfo.GOBILDA_5203_YELLOW_JACKET_MOTOR_435_RPM.ENCODER_RESOLUTION_PPR,
                64.0 / 16.0,
                0.7,
                new Angle(130, false),
                new Angle(-130, false),
                turretStartHeading,
                false
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
        indicatorLightController.setColor(RGBIndicatorLightController.Color.RED);

        chamberDistanceSensor = hardwareMap.get(DistanceSensor.class, "chamber");

        Servo gateServo = hardwareMap.get(Servo.class, "gate");
        gateServoController = new ServoTimerController(gateServo, GATE_SERVO_CLOSED);

        Servo transferServo = hardwareMap.get(Servo.class, "transfer");
        transferServoController = new ServoTimerController(transferServo, TRANSFER_SERVO_DOWN);

        TreeMap<Double, Double> flywheelSpeedByDistanceMap = new TreeMap<>();
        // INCH, RPM
        flywheelSpeedByDistanceMap.put(40.0, 2000.0);
        flywheelSpeedByDistanceMap.put(66.0, 2250.0);
        flywheelSpeedByDistanceMap.put(81.0, 2350.0);
        flywheelSpeedByDistanceMap.put(105.0, 2500.0);
        flywheelSpeedByDistanceMap.put(120.0, 2600.0);
        flywheelSpeedByDistanceMap.put(130.0, 3000.0);
        flywheelVelocityByDistanceInterpolator = new LinearInterpolator(flywheelSpeedByDistanceMap);

        TreeMap<Double, Double> hoodAngleByDistanceMap = new TreeMap<>();
        // INCH, DEGREES
        hoodAngleByDistanceMap.put(40.0, 18.0);
        hoodAngleByDistanceMap.put(66.0, 25.0);
        hoodAngleByDistanceMap.put(81.0, 32.0);
        hoodAngleByDistanceMap.put(105.0, 35.0);
        hoodAngleByDistanceMap.put(120.0, 35.0);
        hoodAngleByDistanceMap.put(130.0, 35.0);
        hoodAngleByDistanceInterpolator = new LinearInterpolator(hoodAngleByDistanceMap);

        TreeMap<Double, Double> timeOfFlightByDistanceMap = new TreeMap<>();
        // INCH, MILLS
        timeOfFlightByDistanceMap.put(40.0, 1000.0);
        timeOfFlightByDistanceMap.put(66.0, 1000.0);
        timeOfFlightByDistanceMap.put(81.0, 1500.0);
        timeOfFlightByDistanceMap.put(105.0, 2100.0);
        timeOfFlightByDistanceMap.put(120.0, 2700.0);
        timeOfFlightByDistanceMap.put(130.0, 3000.0);
        timeOfFlightByDistanceInterpolator = new LinearInterpolator(timeOfFlightByDistanceMap);

        TreeMap<Double, Double> turntableOffsetByDistanceMap = new TreeMap<>();
        // INCH, DEG
        turntableOffsetByDistanceMap.put(40.0, 0.0);
        turntableOffsetByDistanceMap.put(66.0, 0.0);
        turntableOffsetByDistanceMap.put(81.0, 0.0);
        turntableOffsetByDistanceMap.put(105.0, 0.0);
        turntableOffsetByDistanceMap.put(120.0, 0.0);
        turntableOffsetByDistanceMap.put(130.0, -5.0);
        turntableOffsetByDistanceInterpolator = new LinearInterpolator(turntableOffsetByDistanceMap);

        // TODO: Half line point
    }

    @Override
    public void startConfiguration() {

    }

    @Override
    public void update(long deltaTimeMs, TelemetryManager telemetry) {
        super.update(deltaTimeMs, telemetry);

        // GET DATA
        Pose currentPose = follower.getPose();
        Vector velocity = follower.getVelocity();
        double angularVelocity = follower.getAngularVelocity();
        Vector acceleration = follower.getAcceleration();
        double angularAcceleration = 0;

        Pose displacedPose = targetGoal.minus(currentPose);
        double distanceFromGoal = currentPose.distanceFrom(targetGoal);
        Angle goalHeading = new Angle(Math.atan2(displacedPose.getY(), displacedPose.getX()));

        Angle relativeTurntableHeading = turret.turntableController().getHeading();
        JetfireStaticData.lastTurretHeading = relativeTurntableHeading;
        Angle absoluteTurntableHeading = new Angle(relativeTurntableHeading.getAngle(Angle.AngleSystem.SIGNED_180_WRAPPED) + follower.getHeading());

        // PROCESS DATA
        boolean isFlywheelReady = MathUtil.isWithinRange(
                turret.flywheelController().getVelocity(),
                turret.flywheelController().getTargetVelocity(),
                FLYWHEEL_VELOCITY_MARGIN_RPM
        ); // && !flywheelMode.equals(FlywheelMode.OFF);

        boolean isTurntableReady = MathUtil.isWithinRange(
                relativeTurntableHeading.getAngle(Angle.AngleUnit.DEGREES, Angle.AngleSystem.SIGNED_180_WRAPPED),
                turret.turntableController().getTargetHeading().getAngle(Angle.AngleUnit.DEGREES, Angle.AngleSystem.SIGNED_180_WRAPPED),
                TURNTABLE_HEADING_MARGIN_DEG
        );

        boolean isArtifactLoaded = chamberDistanceSensor.getDistance(DistanceUnit.MM) < ARTIFACT_DETECTION_THRESHOLD_MM;
        boolean isCooldownOver = laucnhCooldownTimer.getElapsedTime() > LAUNCH_COOLDOWN_MS;

        isReadyToShoot = isFlywheelReady && isTurntableReady && isArtifactLoaded && isCooldownOver;

        long launchDelay = deltaTimeMs + LAUNCH_DELAY_MS;

        // Lead Computing
        Pose futurePose = PhysicsUtil.predictFuturePose(
                currentPose,
                velocity,
                angularVelocity,
                acceleration,
                angularAcceleration,
                (launchDelay / 1000.0)
        );

        double distanceFromGoalAtFuturePose = futurePose.distanceFrom(targetGoal);
        double timeOfFlightMs = timeOfFlightByDistanceInterpolator.interpolate(distanceFromGoalAtFuturePose);

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

        // UPDATE HARDWARE
        double flywheelSpeed = switch (flywheelMode) {
            case AUTO -> flywheelVelocityByDistanceInterpolator.interpolate(virtualDistanceFromGoal);
            case OFF -> 0.0;
        };

        Angle turntableOffset = new Angle(TUNING ? TURNTABLE_OFFSET_DEG :
                StaticData.allianceColor == OpModeBase.AllianceColor.BLUE ? -turntableOffsetByDistanceInterpolator.interpolate(virtualDistanceFromGoal) : turntableOffsetByDistanceInterpolator.interpolate(virtualDistanceFromGoal),
                false
        ).plus(new Angle(driverTurntableOffset, false), Angle.AngleSystem.SIGNED);
        Angle targetTurntableHeading = autoAimTurntable ? new Angle(virtualGoalHeading.getAngle(Angle.AngleSystem.SIGNED_180_WRAPPED) - follower.getHeading()).plus(turntableOffset, Angle.AngleSystem.SIGNED) : new Angle(0);

        Angle hoodAngle = new Angle(hoodAngleByDistanceInterpolator.interpolate(virtualDistanceFromGoal), false);

        if (!TUNING) {
            turret.update(
                    flywheelSpeed,
                    targetTurntableHeading,
                    hoodAngle
            );
        } else {
            if (!turret.flywheelController().getPidfController().getCoefficients().equals(flywheelPIDFCoefficients)) turret.flywheelController().getPidfController().setCoefficients(flywheelPIDFCoefficients);
            if (!turret.turntableController().getPidfController().getCoefficients().equals(turntablePIDFCoefficients)) turret.turntableController().getPidfController().setCoefficients(turntablePIDFCoefficients);

            turret.update(
                    FLYWHEEL_TARGET_VELOCITY,
                    targetTurntableHeading,
                    new Angle(HOOD_ANGLE, false)
            );
        }

        intakeMotor.setPower(isCooldownOver ? switch (intakeMode) {
            case ON -> reverseIntake ? REVERSE_INTAKE_POWER : INTAKE_POWER;
            case OFF -> 0;
        } : 0);

        indicatorLightController.setColor(isReadyToShoot ? RGBIndicatorLightController.Color.GREEN : RGBIndicatorLightController.Color.RED);

        transferServoController.update();
        gateServoController.update();

        telemetry.addLine("");
        telemetry.addLine("- TRIGGERS -");
        telemetry.addData("isReadyToShoot", isReadyToShoot);
        telemetry.addData("isArtifactLoaded", isArtifactLoaded);
        telemetry.addLine("");
        telemetry.addData("GOAL DISTANCE", distanceFromGoal);
        telemetry.addData("GOAL HEADING", goalHeading.getAngle(Angle.AngleUnit.DEGREES, Angle.AngleSystem.SIGNED_180_WRAPPED));
        telemetry.addLine("");
        telemetry.addLine("- TURNTABLE -");
        telemetry.addData("ABSOLUTE TURNTABLE ERROR", goalHeading.getAngle(Angle.AngleUnit.DEGREES, Angle.AngleSystem.SIGNED_180_WRAPPED) - absoluteTurntableHeading.getAngle(Angle.AngleUnit.DEGREES, Angle.AngleSystem.SIGNED_180_WRAPPED));
        telemetry.addLine("");
        telemetry.addLine("- FLYWHEEL -");
        telemetry.addData("FLYWHEEL VELOCITY RPM", turret.flywheelController().getVelocity());
        telemetry.addData("FLYWHEEL TARGET VELOCITY RPM", turret.flywheelController().getTargetVelocity());
        telemetry.addLine("");
        telemetry.addLine("- HOOD -");
        telemetry.addData("HOOD ANGLE", turret.hoodServoController().getTargetAngle().getAngle(Angle.AngleUnit.DEGREES, Angle.AngleSystem.SIGNED));
        telemetry.addLine("");
        telemetry.addLine("- LEAD COMPUTING -");
        telemetry.addData("Future Pose X", futurePose.getX());
        telemetry.addData("Future Pose Y", futurePose.getY());
    }

    public void shoot() {
        gateServoController.setPosition(GATE_SERVO_OPEN, GATE_SERVO_TIME_MS, GATE_SERVO_CLOSED);
        transferServoController.setPosition(TRANSFER_SERVO_UP, TRANSFER_SERVO_TIME_MS, TRANSFER_SERVO_DOWN);

        laucnhCooldownTimer.resetTimer();
    }

    public void setIntakeMode(IntakeMode intakeMode) {
        this.intakeMode = intakeMode;
    }

    public IntakeMode getIntakeMode() {
        return intakeMode;
    }

    public boolean isReverseIntake() {
        return reverseIntake;
    }

    public void setReverseIntake(boolean reverseIntake) {
        this.reverseIntake = reverseIntake;
    }

    public boolean isAutoAimTurntable() {
        return autoAimTurntable;
    }

    public void setAutoAimTurntable(boolean autoAimTurntable) {
        this.autoAimTurntable = autoAimTurntable;
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

    public double getDriverTurntableOffset() {
        return driverTurntableOffset;
    }

    public void setDriverTurntableOffset(double driverTurntableOffset) {
        this.driverTurntableOffset = driverTurntableOffset;
    }

    @Override
    protected FieldType instantiateFieldType() {
        return FieldType.SQUARE_INVERTED_ALLIANCE;
    }

    @Override
    public Follower instantiateFollower(HardwareMap hardwareMap) {
        return Constants.createJetfireFollower(hardwareMap);
    }

    @Override
    public Limelight3A instantiateLimelight3A(HardwareMap hardwareMap) {
        return null;
    }
}
