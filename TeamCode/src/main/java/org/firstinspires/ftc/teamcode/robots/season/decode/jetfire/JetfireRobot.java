package org.firstinspires.ftc.teamcode.robots.season.decode.jetfire;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.control.KalmanFilter;
import com.pedropathing.control.KalmanFilterParameters;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.hardware.controllers.StateMachine;
import org.firstinspires.ftc.teamcode.hardware.controllers.digital.GoBildaLaserDistanceSensorDigital;
import org.firstinspires.ftc.teamcode.hardware.controllers.lights.Prism.Color;
import org.firstinspires.ftc.teamcode.hardware.controllers.lights.Prism.GoBildaPrismDriver;
import org.firstinspires.ftc.teamcode.hardware.controllers.lights.Prism.PrismAnimations;
import org.firstinspires.ftc.teamcode.hardware.controllers.motor.TurntableMotorController;
import org.firstinspires.ftc.teamcode.hardware.controllers.motor.VelocityMotorController;
import org.firstinspires.ftc.teamcode.hardware.controllers.servo.AngleServoController;
import org.firstinspires.ftc.teamcode.hardware.controllers.servo.RGBIndicatorLightController;
import org.firstinspires.ftc.teamcode.hardware.controllers.servo.ServoTimerController;
import org.firstinspires.ftc.teamcode.hardware.controllers.subsystems.Turret;
import org.firstinspires.ftc.teamcode.hardware.drivetrain.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.hardware.vision.LimelightHandler;
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
    public static PIDFCoefficients flywheelPIDFCoefficients = new PIDFCoefficients(0.007, 0, 0, 0.0002);

    // TURNTABLE
    public static PIDFCoefficients turntablePIDFCoefficients = new PIDFCoefficients(0.051, 0, 0.0012, 0);
    public static double CLOSE_ZONE_TURNTABLE_OFFSET_DEG = 2;
    public static double FAR_ZONE_TURNTABLE_OFFSET_DEG = -1; // neg toward opp alliance, pos toward your alliance

    // INTAKE
    DcMotorEx intakeMotor;
    public static double INTAKE_POWER = 1;
    public static double REVERSE_INTAKE_POWER = -0.6;
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

    // TRANSFER SERVO
    ServoTimerController transferServoController;
    public static double TRANSFER_SERVO_UP = 0.4;
    public static double TRANSFER_SERVO_DOWN = 0.67;
    public static int TRANSFER_SERVO_TIME_MS = 200; // TODO: Decrease

    // GATE SERVO
    ServoTimerController gateServoController;
    public static double GATE_SERVO_OPEN = 0.6;
    public static double GATE_SERVO_CLOSED = 0.8;
    public static int GATE_SERVO_TIME_MS = 300; // INCREASE A BIT

    // ARTIFACT CHAMBER DETECTION
    DistanceSensor chamberDistanceSensor;
    public static KalmanFilterParameters chamberFilterParameters = new KalmanFilterParameters(0.1, 0.1);
    private KalmanFilter chamberKalmanFilter = new KalmanFilter(chamberFilterParameters);
    public static double ARTIFACT_DETECTION_THRESHOLD_MM = 100;


    GoBildaLaserDistanceSensorDigital chamberSensor;

    // COOLDOWN
    Timer laucnhCooldownTimer = new Timer();
    public static int LAUNCH_COOLDOWN_MS = 400; // 400
    public static int INTAKE_COOLDOWN_MS = 150; // 150
    // 166 for half second, 250 for 3/4 second

    // MARGINS & DELAYS
    public static double FLYWHEEL_VELOCITY_MARGIN_RPM = 60;
    public static double TURNTABLE_HEADING_MARGIN_DEG = 2;

    public static long LAUNCH_DELAY_MS = 80;

    // LIGHTS
    RGBIndicatorLightController indicatorLightController;
    GoBildaPrismDriver prism;

    Timer indicateTimer = new Timer();
    private static final int INDICATE_TIME_MS = 500;
    RGBIndicatorLightController.Color tempIndicateColor = RGBIndicatorLightController.Color.OFF;

    // TUNING
    public static boolean TUNING = false;
    public static double FLYWHEEL_TARGET_VELOCITY = 0;
    public static double HOOD_ANGLE = 16;
    public static double TURNTABLE_OFFSET_DEG = 0;

    private final static double FAR_ZONE_X_THRESHOLD = 24;

    // RUNTIME
    public static Pose targetGoal;
    private boolean isReadyToShoot = false;
    private boolean isInFarZone = false;
    private boolean isFlywheelReady = false;
    private boolean isArtifactLoaded;

    private final StateMachine autoFire = new StateMachine();

    // LimeLight
    private LimelightHandler limelightHandler;


    List<LynxModule> allLynxModules;

    @Override
    public void init(HardwareMap hardwareMap, Pose startPose, OpModeBase.AllianceColor allianceColor) {
        super.init(hardwareMap, startPose, allianceColor);

        targetGoal = new Pose(-67, allianceColor == OpModeBase.AllianceColor.RED ? 67 : -67);

        TreeMap<Double, Double> flywheelSpeedByDistanceMap = new TreeMap<>();
        // INCH, RPM
        flywheelSpeedByDistanceMap.put(33.0, 2000.0);
        flywheelSpeedByDistanceMap.put(59.0, 2200.0);
        flywheelSpeedByDistanceMap.put(74.0, 2400.0);
        flywheelSpeedByDistanceMap.put(98.0, 2500.0);
        flywheelSpeedByDistanceMap.put(113.0, 2600.0);
        flywheelSpeedByDistanceMap.put(123.0, 3000.0);
        flywheelVelocityByDistanceInterpolator = new LinearInterpolator(flywheelSpeedByDistanceMap);

        TreeMap<Double, Double> hoodAngleByDistanceMap = new TreeMap<>();
        // INCH, DEGREES
        hoodAngleByDistanceMap.put(33.0, 24.0);
        hoodAngleByDistanceMap.put(59.0, 31.0);
        hoodAngleByDistanceMap.put(74.0, 38.0);
        hoodAngleByDistanceMap.put(98.0, 41.0);
        hoodAngleByDistanceMap.put(113.0, 41.0);
        hoodAngleByDistanceMap.put(123.0, 41.0);
        hoodAngleByDistanceInterpolator = new LinearInterpolator(hoodAngleByDistanceMap);

        TreeMap<Double, Double> timeOfFlightByDistanceMap = new TreeMap<>();
        // INCH, MILLS
        timeOfFlightByDistanceMap.put(33.0, 500.0);
        timeOfFlightByDistanceMap.put(59.0, 640.0);
        timeOfFlightByDistanceMap.put(74.0, 730.0);
        timeOfFlightByDistanceMap.put(98.0, 750.0);
        timeOfFlightByDistanceMap.put(113.0, 760.0);
        timeOfFlightByDistanceMap.put(123.0, 770.0);
        timeOfFlightByDistanceInterpolator = new LinearInterpolator(timeOfFlightByDistanceMap);


        // TODO: State machine class
//        for (int i = 0; i <= 2; i++) {
//            autoFire.addStep(
//                    () -> isReadyToShoot,
//                    this::fire
//            );
//        }
    }

    @Override
    public void initHardware(HardwareMap hardwareMap) {
        allLynxModules = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allLynxModules) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        VelocityMotorController flywheelController = new VelocityMotorController(
                hardwareMap.get(DcMotorEx.class, "flywheel"),
                DcMotorSimple.Direction.REVERSE,
                flywheelPIDFCoefficients,
                HardwareInfo.GOBILDA_5203_YELLOW_JACKET_MOTOR_6000_RPM.ENCODER_RESOLUTION_PPR,
                1,
                1
        );

        Angle turretStartHeading = Optional.ofNullable(JetfireStaticData.lastTurretHeading)
                .orElse(new Angle(0));

        TurntableMotorController turntableController = new TurntableMotorController(
                hardwareMap.get(DcMotorEx.class, "turntable"),
                DcMotorSimple.Direction.FORWARD,
                turntablePIDFCoefficients,
                HardwareInfo.GOBILDA_5203_YELLOW_JACKET_MOTOR_435_RPM.ENCODER_RESOLUTION_PPR,
                64.0 / 16.0,
                0.75,
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
                new Angle(22, false),
                new Angle(42, false),
                new Angle(22, false)
        );

        turret = new Turret(
                flywheelController,
                turntableController,
                hoodController
        );

        intakeMotor = hardwareMap.get(DcMotorEx.class, "intake");
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        Servo gateServo = hardwareMap.get(Servo.class, "gate");
        gateServoController = new ServoTimerController(gateServo, GATE_SERVO_CLOSED);

        Servo transferServo = hardwareMap.get(Servo.class, "transfer");
        transferServoController = new ServoTimerController(transferServo, TRANSFER_SERVO_DOWN);

        chamberDistanceSensor = hardwareMap.get(DistanceSensor.class, "chamber");
        //chamberSensor = new GoBildaLaserDistanceSensorDigital(hardwareMap.get(DigitalChannel.class, "laser-digital-input"));

        Limelight3A limelight3A = hardwareMap.get(Limelight3A.class, "limelight");
        limelightHandler = new LimelightHandler(limelight3A);
        limelightHandler.init(0);

        indicatorLightController = new RGBIndicatorLightController(hardwareMap.get(Servo.class, "indicator"));
        indicatorLightController.setColor(StaticData.allianceColor == OpModeBase.AllianceColor.RED ? RGBIndicatorLightController.Color.RED : RGBIndicatorLightController.Color.BLUE);

        prism = hardwareMap.get(GoBildaPrismDriver.class,"prism");
        PrismAnimations.DroidScan droidScan = new PrismAnimations.DroidScan();
        droidScan.setPrimaryColor(StaticData.allianceColor == OpModeBase.AllianceColor.RED ? Color.RED : Color.BLUE);
        droidScan.setSpeed(0.05f);
        droidScan.setEyeWidth(4);

        droidScan.setIndexes(1, 17);
        prism.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_0, droidScan);
        droidScan.setIndexes(18, 36);
        prism.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_1, droidScan);
    }

    @Override
    public void startConfiguration() {
        // TODO: Init servo pos here
    }

    @Override
    public void update(long deltaTimeMs, TelemetryManager telemetry) {
        for (LynxModule hub : allLynxModules) {
            hub.clearBulkCache();
        }

        Pose currentPose;
        Vector velocity = follower.getVelocity();

        Pose pinpointPose = follower.getPose();
        Pose limelightPose = limelightHandler.getPose();

        if (limelightPose == null) {
            currentPose = pinpointPose;
        } else {
            currentPose = limelightPose;
            follower.setPose(limelightPose);
        }

        double headingDeg = Math.toDegrees(pinpointPose.getHeading());

        Angle relativeTurntableHeading = turret.turntableController().getHeading();
        //Angle absoluteTurntableHeading = new Angle(relativeTurntableHeading.getAngle(Angle.AngleNormalization.BIPOLAR) + follower.getHeading());

        isInFarZone = currentPose.getX() > FAR_ZONE_X_THRESHOLD;

        isFlywheelReady = MathUtil.isWithinRange(
                turret.flywheelController().getVelocity(),
                turret.flywheelController().getTargetVelocity(),
                FLYWHEEL_VELOCITY_MARGIN_RPM
        ) && !flywheelMode.equals(FlywheelMode.OFF);

        boolean isTurntableReady = MathUtil.isWithinRange(
                relativeTurntableHeading.getAngle(Angle.AngleUnit.DEGREES, Angle.AngleNormalization.BIPOLAR),
                turret.turntableController().getTargetHeading().getAngle(Angle.AngleUnit.DEGREES, Angle.AngleNormalization.BIPOLAR),
                TURNTABLE_HEADING_MARGIN_DEG
        );

        double chamberDistanceMM = chamberDistanceSensor.getDistance(DistanceUnit.MM);
        isArtifactLoaded = chamberDistanceMM < ARTIFACT_DETECTION_THRESHOLD_MM;
        // isArtifactLoaded = chamberSensor.isObjectDetected();

        boolean isCooldownOver = laucnhCooldownTimer.getElapsedTime() > LAUNCH_COOLDOWN_MS;
        boolean isIntakeCooldownOver = laucnhCooldownTimer.getElapsedTime() > INTAKE_COOLDOWN_MS;

        isReadyToShoot = isTurntableReady && isArtifactLoaded && isCooldownOver; // isFlywheelReady

        // Lead Computing
        double launchDelaySec = (deltaTimeMs + LAUNCH_DELAY_MS) / 1000.0;

        Pose predictedFuturePose = PhysicsUtil.predictFuturePose(
                currentPose,
                velocity,
                launchDelaySec
        );

        double distanceFromGoalAtFuturePose = predictedFuturePose.distanceFrom(targetGoal);
        double timeOfFlightSec = timeOfFlightByDistanceInterpolator.interpolate(distanceFromGoalAtFuturePose) / 1000.0;

        Pose virtualGoal = PhysicsUtil.predictFuturePose(
                targetGoal,
                velocity.times(-1),
                timeOfFlightSec
        );

        //follower.getPoseHistory().

        double virtualDistanceFromGoal = predictedFuturePose.distanceFrom(virtualGoal);
//        Pose virtualDisplacedPose = virtualGoal.minus(predictedFuturePose);
//        Angle virtualGoalHeading = new Angle(Math.atan2(virtualDisplacedPose.getY(), virtualDisplacedPose.getX()));
        Angle virtualGoalHeading = new Angle(MathUtil.bearingTo(predictedFuturePose, virtualGoal));

        // UPDATE HARDWARE
        double flywheelSpeed = switch (flywheelMode) {
            case AUTO -> flywheelVelocityByDistanceInterpolator.interpolate(virtualDistanceFromGoal);
            case OFF -> 0.0;
        };

        double turntableZoneOffsetDeg = isInFarZone ? FAR_ZONE_TURNTABLE_OFFSET_DEG : CLOSE_ZONE_TURNTABLE_OFFSET_DEG;
        Angle turntableZoneOffset = new Angle(
                turntableZoneOffsetDeg * ((StaticData.allianceColor == OpModeBase.AllianceColor.BLUE) ? 1 : -1),
                false
        );

        Angle turntableHeading = virtualGoalHeading.minus(new Angle(currentPose.getHeading()), Angle.AngleNormalization.NONE).plus(turntableZoneOffset, Angle.AngleNormalization.NONE);
        Angle hoodAngle = new Angle(hoodAngleByDistanceInterpolator.interpolate(virtualDistanceFromGoal), false);

        if (!TUNING) {
            turret.update(
                    flywheelSpeed,
                    autoAimTurntable ? turntableHeading : new Angle(0),
                    hoodAngle
            );
        } else {
            if (!turret.flywheelController().getPidfController().getCoefficients().equals(flywheelPIDFCoefficients)) turret.flywheelController().getPidfController().setCoefficients(flywheelPIDFCoefficients);
            if (!turret.turntableController().getPidfController().getCoefficients().equals(turntablePIDFCoefficients)) turret.turntableController().getPidfController().setCoefficients(turntablePIDFCoefficients);

            turret.update(
                    FLYWHEEL_TARGET_VELOCITY,
                    new Angle(TURNTABLE_OFFSET_DEG, false),
                    new Angle(HOOD_ANGLE, false)
            );
        }

        intakeMotor.setPower(isIntakeCooldownOver ? switch (intakeMode) {
            case ON -> reverseIntake ? REVERSE_INTAKE_POWER : INTAKE_POWER;
            case OFF -> 0;
        } : 0);

        if (OpModeBase.getOpModeElapsedTimeSeconds() > 110) {
            indicatorLightController.setColor(RGBIndicatorLightController.Color.WHITE);
        } else if (indicateTimer.getElapsedTime() < INDICATE_TIME_MS) {
            indicatorLightController.setColor(tempIndicateColor);
        } else {
            if (!isTurntableReady) {
                indicatorLightController.setColor(RGBIndicatorLightController.Color.ORANGE);
            } else if (!isArtifactLoaded) {
                indicatorLightController.setColor(RGBIndicatorLightController.Color.RED);
            } else {
                indicatorLightController.setColor(RGBIndicatorLightController.Color.GREEN);
            }
        }

        // UPDATE HARDWARE
        super.update(deltaTimeMs, telemetry);

        limelightHandler.update(headingDeg);

        transferServoController.update();
        gateServoController.update();
        // autoFire.update();


        // UPDATE STATIC
        JetfireStaticData.lastTurretHeading = relativeTurntableHeading;

        // TODO: LL primary odo (sensor fusion problems while moving, Kalman, or poseTracker for latency)
        // TODO: Indicator light for endgame park time
        // TODO: Rumble on controllers
        // TODO: Should we filter and use acceleration? Nah Michael Jackson
        // TODO: bare motor, new distance sensor, faster kick server for transfer, poinpoint v2
        // INTAKE that goes all the way to edge
        // breakpads
        // TODO: Double servos for kicker
        // TODO: Switch to 2mm Timing belt over chain?
        // Manualy reset lynx
        // TODO: Dual motor class for flywheel


        // - UPDATE TELEMETRY -

        telemetry.addLine("- LIMELIGHT -");
        telemetry.addData("Active", limelightPose != null);
        telemetry.addLine("");
        telemetry.addLine("- OFFSETS -");
        telemetry.addData("isInFarZone", isInFarZone);
        telemetry.addData("Close Turntable Offset", CLOSE_ZONE_TURNTABLE_OFFSET_DEG);
        telemetry.addData("Far Turntable Offset", FAR_ZONE_TURNTABLE_OFFSET_DEG);
        telemetry.addLine("");
        telemetry.addLine("- CONDITIONS -");
        telemetry.addData("isReadyToShoot", isReadyToShoot);
        telemetry.addData("isArtifactLoaded", isArtifactLoaded);
        telemetry.addData("isTurntableReady", isTurntableReady);
        telemetry.addData("isFlywheelReady", isFlywheelReady);
        telemetry.addData("isCooldownOver", isCooldownOver);
        telemetry.addLine("");
        telemetry.addLine("- TURNTABLE -");
        telemetry.addData("Relative Heading", relativeTurntableHeading.getAngle(Angle.AngleUnit.DEGREES, Angle.AngleNormalization.NONE));
        //telemetry.addData("Absolute Heading", absoluteTurntableHeading.getAngle(Angle.AngleUnit.DEGREES, Angle.AngleNormalization.NONE));
        //telemetry.addData("Absolute Error", goalHeading.getAngle(Angle.AngleUnit.DEGREES, Angle.AngleNormalization.BIPOLAR) - absoluteTurntableHeading.getAngle(Angle.AngleUnit.DEGREES, Angle.AngleNormalization.BIPOLAR));
        telemetry.addLine("");
        telemetry.addLine("- FLYWHEEL -");
        telemetry.addData("FLYWHEEL VELOCITY RPM", turret.flywheelController().getVelocity());
        telemetry.addData("FLYWHEEL TARGET VELOCITY RPM", turret.flywheelController().getTargetVelocity());
        telemetry.addLine("");
        telemetry.addLine("- HOOD -");
        telemetry.addData("HOOD ANGLE", turret.hoodServoController().getTargetAngle().getAngle(Angle.AngleUnit.DEGREES, Angle.AngleNormalization.NONE));


        if (TUNING) {
            Angle goalHeading = new Angle(MathUtil.bearingTo(currentPose, targetGoal));
            telemetry.addLine("");
            telemetry.addLine("- TUNING DATA -");
            telemetry.addData("Absolute Heading Toward Goal", goalHeading.getAngle(Angle.AngleUnit.DEGREES, Angle.AngleNormalization.BIPOLAR));
            telemetry.addLine("");

            //Pose displacedPose = targetGoal.minus(currentPose);
            double distanceFromGoal = currentPose.distanceFrom(targetGoal);
//            Angle goalHeading = new Angle(Math.atan2(displacedPose.getY(), displacedPose.getX()));

            telemetry.addLine("- LEAD COMPUTING & LUTs -");
            telemetry.addData("Future Pose X", predictedFuturePose.getX());
            telemetry.addData("Future Pose Y", predictedFuturePose.getY());
            telemetry.addData("Time of Flight (sec)", timeOfFlightSec);
            telemetry.addData("Goal Distance", distanceFromGoal);
            telemetry.addLine("");
            telemetry.addLine("- DISTANCE SENSOR -");
            telemetry.addData("Chamber Distance", chamberDistanceMM);


        }
    }

    public void fire() {
        gateServoController.setPosition(GATE_SERVO_OPEN, GATE_SERVO_TIME_MS, GATE_SERVO_CLOSED);
        transferServoController.setPosition(TRANSFER_SERVO_UP, TRANSFER_SERVO_TIME_MS, TRANSFER_SERVO_DOWN);

        laucnhCooldownTimer.resetTimer();
    }

    public void startAllSubsystems() {
        setFlywheelMode(JetfireRobot.FlywheelMode.AUTO);
        setIntakeMode(JetfireRobot.IntakeMode.ON);
        setAutoAimTurntable(true);
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

    public LimelightHandler getLimelightHandler() {
        return limelightHandler;
    }

    public StateMachine getAutoFire() {
        return autoFire;
    }

    public KalmanFilter getChamberKalmanFilter() {
        return chamberKalmanFilter;
    }

    public void setChamberKalmanFilter(KalmanFilter chamberKalmanFilter) {
        this.chamberKalmanFilter = chamberKalmanFilter;
    }

    public Turret getTurret() {
        return turret;
    }

    public void indicate(RGBIndicatorLightController.Color color) {
        tempIndicateColor = color;
        indicateTimer.resetTimer();
    }

    public boolean isInFarZone() {
        return isInFarZone;
    }

    public boolean isFlywheelReady() {
        return isFlywheelReady;
    }

    public boolean isArtifactLoaded() {
        return isArtifactLoaded;
    }

    @Override
    protected FieldType instantiateFieldType() {
        return FieldType.SQUARE_INVERTED_ALLIANCE;
    }

    @Override
    public Follower instantiateFollower(HardwareMap hardwareMap) {
        return Constants.createJetfireFollower(hardwareMap);
    }
}