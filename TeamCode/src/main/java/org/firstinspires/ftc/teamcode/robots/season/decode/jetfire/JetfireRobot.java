package org.firstinspires.ftc.teamcode.robots.season.decode.jetfire;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.hardware.controllers.StateMachine;
import org.firstinspires.ftc.teamcode.hardware.controllers.digital.GoBildaLaserDistanceSensorDigital;
import org.firstinspires.ftc.teamcode.hardware.controllers.lights.Prism.Color;
import org.firstinspires.ftc.teamcode.hardware.controllers.lights.Prism.GoBildaPrismDriver;
import org.firstinspires.ftc.teamcode.hardware.controllers.lights.Prism.PrismAnimations;
import org.firstinspires.ftc.teamcode.hardware.controllers.motor.BasicMotorController;
import org.firstinspires.ftc.teamcode.hardware.controllers.motor.TurntableMotorController;
import org.firstinspires.ftc.teamcode.hardware.controllers.servo.AngleServoController;
import org.firstinspires.ftc.teamcode.hardware.controllers.servo.RGBIndicatorLightController;
import org.firstinspires.ftc.teamcode.hardware.controllers.servo.ServoTimerController;
import org.firstinspires.ftc.teamcode.hardware.controllers.motor.DualMotorVelocityController;
import org.firstinspires.ftc.teamcode.hardware.controllers.subsystems.Turret;
import org.firstinspires.ftc.teamcode.hardware.drivetrain.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.hardware.vision.LimelightController;
import org.firstinspires.ftc.teamcode.robots.base.RobotBase;
import org.firstinspires.ftc.teamcode.robots.base.StaticData;
import org.firstinspires.ftc.teamcode.robots.base.opmodes.OpModeBase;
import org.firstinspires.ftc.teamcode.util.LinearInterpolator;
import org.firstinspires.ftc.teamcode.util.PoseHistory;
import org.firstinspires.ftc.teamcode.util.info.HardwareInfo;
import org.firstinspires.ftc.teamcode.util.math.Angle;
import org.firstinspires.ftc.teamcode.util.math.MathUtil;
import static org.firstinspires.ftc.teamcode.robots.season.decode.jetfire.JetFireConstants.*;

import java.util.Optional;
import java.util.TreeMap;
import java.util.concurrent.TimeUnit;

@Configurable
public class JetfireRobot extends RobotBase {
    private Turret<DualMotorVelocityController> turret;
    BasicMotorController intakeMotorController;

    ServoTimerController transferServoController;
    ServoTimerController gateServoController;

    GoBildaLaserDistanceSensorDigital chamberSensor;

    RGBIndicatorLightController indicatorLightController;
    GoBildaPrismDriver prism;

    private boolean autoAimTurntable = false;

    private boolean isFlywheelOn = false;

    private boolean isIntakeOn = false;
    private boolean reverseIntake = false;

    public static double CLOSE_ZONE_TURNTABLE_OFFSET_DEG = 2;
    public static double FAR_ZONE_TURNTABLE_OFFSET_DEG = -1; // neg toward opp alliance, pos toward your alliance

    // LUTs
    private LinearInterpolator flywheelVelocityByDistanceInterpolator;
    private LinearInterpolator hoodAngleByDistanceInterpolator;
    private LinearInterpolator timeOfFlightByDistanceInterpolator;

    // COOLDOWN
    Timer laucnhCooldownTimer = new Timer();

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
    private LimelightController limelightController;

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
        DcMotorEx rightFlywheelMotor = hardwareMap.get(DcMotorEx.class, "right-flywheel");
        rightFlywheelMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        DcMotorEx leftFlywheelMotor = hardwareMap.get(DcMotorEx.class, "left-flywheel");
        leftFlywheelMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        DualMotorVelocityController flywheelController = new DualMotorVelocityController(
                rightFlywheelMotor,
                leftFlywheelMotor,
                FLYWHEEL_NAME,
                FLYWHEEL_PIDF_COEFFICIENTS,
                HardwareInfo.GOBILDA_5203_YELLOW_JACKET_MOTOR_6000_RPM.ENCODER_RESOLUTION_PPR,
                FLYWHEEL_GEAR_RATIO,
                FLYWHEEL_MAX_POWER
        );

        Angle turretStartHeading = Optional.ofNullable(JetfireStaticData.lastTurretHeading)
                .orElse(new Angle(0));

        DcMotorEx turntableMotor = hardwareMap.get(DcMotorEx.class, "turntable");

        TurntableMotorController turntableController = new TurntableMotorController(
                turntableMotor,
                TURNTABLE_NAME,
                TURNTABLE_PIDF_COEFFICIENTS,
                HardwareInfo.GOBILDA_5203_YELLOW_JACKET_MOTOR_435_RPM.ENCODER_RESOLUTION_PPR,
                TURNTABLE_TOTAL_GEAR_RATIO,
                TURNTABLE_MAX_POWER,
                new Angle(TURNTABLE_HARD_STOP, false),
                new Angle(-TURNTABLE_HARD_STOP, false),
                false,
                turretStartHeading
        );

        Servo hoodServo = hardwareMap.get(Servo.class, "hood");
        hoodServo.setDirection(Servo.Direction.REVERSE);

        AngleServoController hoodController = new AngleServoController(
                hoodServo,
                HOOD_NAME,
                HOOD_TOTAL_ROTATION,
                HOOD_TOTAL_GEAR_RATIO,
                MIN_HOOD_ANGLE,
                MAX_HOOD_ANGLE
        );

        turret = new Turret<>(flywheelController, turntableController, hoodController);

        DcMotorEx intakeMotor;

        intakeMotor = hardwareMap.get(DcMotorEx.class, "intake");
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        intakeMotorController = new BasicMotorController(
                intakeMotor,
                INTAKE_NAME,
                INTAKE_PIDF_COEFFICIENTS,
                HardwareInfo.GOBILDA_5203_YELLOW_JACKET_MOTOR_1150_RPM.ENCODER_RESOLUTION_PPR,
                INTAKE_TOTAL_GEAR_RATIO,
                INTAKE_MAX_POWER
        );

        Servo gateServo = hardwareMap.get(Servo.class, "gate");
        gateServoController = new ServoTimerController(gateServo, GATE_SERVO_CLOSED);

        Servo transferServo = hardwareMap.get(Servo.class, "transfer");
        transferServoController = new ServoTimerController(transferServo, TRANSFER_SERVO_DOWN);

        chamberSensor = new GoBildaLaserDistanceSensorDigital(hardwareMap.get(DigitalChannel.class, "laser-digital-input"));

        Limelight3A limelight3A = hardwareMap.get(Limelight3A.class, "limelight");
        limelightController = new LimelightController(limelight3A, "Limelight");
        limelightController.init(0);

        Servo indicator = hardwareMap.get(Servo.class, "indicator");
        indicatorLightController = new RGBIndicatorLightController(indicator, "Indicator");
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

    PoseHistory poseHistory = new PoseHistory(5);
    @Override
    public void update(long deltaTimeMs, TelemetryManager telemetry) {
        Pose pinpointPose = follower.getPose();
        Pose limelightPose = limelightController.hasResult() ? limelightController.getPose() : pinpointPose;

        long limelightLatency = (long) limelightController.getLatencyMs();

        // either odo has moved past ll
        // or odo has drifeted and ll is correct
        if (pinpointPose.distanceFrom(limelightPose) > 2) {
            Pose pinpointPoseAtLimelightCapture = poseHistory.getPoseAt(limelightLatency);

            follower.poseTracker.setXOffset(limelightPose.getX() - pinpointPoseAtLimelightCapture.getX());
            follower.poseTracker.setYOffset(limelightPose.getY() - pinpointPoseAtLimelightCapture.getY());
        }

        Pose currentPose = follower.getPose();
        Vector velocity = follower.getVelocity();

        double headingDeg = Math.toDegrees(pinpointPose.getHeading());

        Angle relativeTurntableHeading = turret.turntableController().getHeading();
        double relativeTurntableHeadingDeg = relativeTurntableHeading.getAngle(Angle.AngleUnit.DEGREES, Angle.AngleNormalization.BIPOLAR);
        double relativeTurntableTargetHeadingDeg = turret.turntableController().getTargetHeading().getAngle(Angle.AngleUnit.DEGREES, Angle.AngleNormalization.BIPOLAR);

        //Angle absoluteTurntableHeading = new Angle(relativeTurntableHeading.getAngle(Angle.AngleNormalization.BIPOLAR) + follower.getHeading());

        isInFarZone = currentPose.getX() > FAR_ZONE_X_THRESHOLD;

        double flywheelVelocity = turret.flywheelController().getVelocity();
        double flywheelTargetVelocity = turret.flywheelController().getTargetVelocity();

        isFlywheelReady = MathUtil.isWithinRange(flywheelVelocity, flywheelTargetVelocity, FLYWHEEL_VELOCITY_MARGIN_RPM) && !isFlywheelOn;

        boolean isTurntableReady = MathUtil.isWithinRange(relativeTurntableHeadingDeg, relativeTurntableTargetHeadingDeg, TURNTABLE_HEADING_MARGIN_DEG);

        isArtifactLoaded = chamberSensor.isObjectDetected();

        boolean isCooldownOver = laucnhCooldownTimer.getElapsedTime() > LAUNCH_COOLDOWN_MS;
        boolean isIntakeCooldownOver = laucnhCooldownTimer.getElapsedTime() > INTAKE_COOLDOWN_MS;

        isReadyToShoot = isTurntableReady && isArtifactLoaded && isCooldownOver; // isFlywheelReady

        // Lead Computing
        long totalLaunchDelayMills = deltaTimeMs + LEAD_COMPUTING_TRANSFER_DELAY_MS;
        double launchDelaySec = TimeUnit.MILLISECONDS.convert(totalLaunchDelayMills, TimeUnit.SECONDS);
                //(deltaTimeMs + LEAD_COMPUTING_TRANSFER_DELAY_MS) / 1000.0;

        Pose predictedFuturePose = MathUtil.predictFuturePose(currentPose, velocity, launchDelaySec);

        double distanceFromGoalAtFuturePose = predictedFuturePose.distanceFrom(targetGoal);

        long timeOfFlightMills = (long) timeOfFlightByDistanceInterpolator.interpolate(distanceFromGoalAtFuturePose);
        double timeOfFlightSec = TimeUnit.MILLISECONDS.convert(timeOfFlightMills, TimeUnit.SECONDS);

        Pose virtualGoal = MathUtil.predictFuturePose(targetGoal, velocity.times(-1), timeOfFlightSec);

        double virtualDistanceFromGoal = predictedFuturePose.distanceFrom(virtualGoal);
        Angle virtualGoalHeading = new Angle(MathUtil.bearingTo(predictedFuturePose, virtualGoal));

        // UPDATE HARDWARE
        double flywheelSpeed = isFlywheelOn ? flywheelVelocityByDistanceInterpolator.interpolate(virtualDistanceFromGoal) : 0;

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
            if (!turret.flywheelController().getPidfController().getCoefficients().equals(FLYWHEEL_PIDF_COEFFICIENTS)) turret.flywheelController().getPidfController().setCoefficients(FLYWHEEL_PIDF_COEFFICIENTS);
            if (!turret.turntableController().getPidfController().getCoefficients().equals(TURNTABLE_PIDF_COEFFICIENTS)) turret.turntableController().getPidfController().setCoefficients(TURNTABLE_PIDF_COEFFICIENTS);

            turret.update(
                    FLYWHEEL_TARGET_VELOCITY,
                    new Angle(TURNTABLE_OFFSET_DEG, false),
                    new Angle(HOOD_ANGLE, false)
            );
        }

        double intakePower = 0.0;

        if (isIntakeOn && isIntakeCooldownOver) {
            intakePower = reverseIntake ? REVERSE_INTAKE_POWER : INTAKE_POWER;
        }

        if (OpModeBase.getOpModeElapsedTimeSeconds() > 110) {
            indicatorLightController.setColor(RGBIndicatorLightController.Color.WHITE);
        } else if (!isTurntableReady) {
            indicatorLightController.setColor(RGBIndicatorLightController.Color.ORANGE);
        } else if (!isArtifactLoaded) {
            indicatorLightController.setColor(RGBIndicatorLightController.Color.RED);
        } else {
            indicatorLightController.setColor(RGBIndicatorLightController.Color.GREEN);
        }

        // UPDATE HARDWARE
        super.update(deltaTimeMs, telemetry);

        chamberSensor.update();

        limelightController.updateRobotHeading(headingDeg);
        limelightController.update();

        transferServoController.update();
        gateServoController.update();

        intakeMotorController.setPower(intakePower);
        // autoFire.update();


        // UPDATE STATIC
        JetfireStaticData.lastTurretHeading = relativeTurntableHeading;

        // TODO: LL primary odo (sensor fusion problems while moving, Kalman, or poseTracker for latency)
        // TODO: Indicator light for endgame park time
        // TODO: Rumble on controllers
        // INTAKE that goes all the way to edge

        // - UPDATE TELEMETRY -

        telemetry.addLine("- CONDITIONS -");
        telemetry.addData("isReadyToShoot", isReadyToShoot);
        telemetry.addData("isArtifactLoaded", isArtifactLoaded);
        telemetry.addData("isTurntableReady", isTurntableReady);
        telemetry.addData("isFlywheelReady", isFlywheelReady);
        telemetry.addLine("");

        telemetry.addLine("- OFFSETS -");
        telemetry.addData("isInFarZone", isInFarZone);
        telemetry.addData("Close Turntable Offset", CLOSE_ZONE_TURNTABLE_OFFSET_DEG);
        telemetry.addData("Far Turntable Offset", FAR_ZONE_TURNTABLE_OFFSET_DEG);

        limelightController.addTelemetry(telemetry);
        turret.turntableController().addTelemetry(telemetry);
        turret.flywheelController().addTelemetry(telemetry);
        turret.hoodServoController().addTelemetry(telemetry);

        if (TUNING) {
            Angle goalHeading = new Angle(MathUtil.bearingTo(currentPose, targetGoal));
            double distanceFromGoal = currentPose.distanceFrom(targetGoal);

            telemetry.addLine("");
            telemetry.addLine("- TUNING DATA -");
            telemetry.addData("Absolute Heading Toward Goal", goalHeading.getAngle(Angle.AngleUnit.DEGREES, Angle.AngleNormalization.BIPOLAR));
            telemetry.addData("Distance From Goal", distanceFromGoal);
            telemetry.addLine("");

            telemetry.addLine("- LEAD COMPUTING & LUTs -");
            telemetry.addData("Future Pose X", predictedFuturePose.getX());
            telemetry.addData("Future Pose Y", predictedFuturePose.getY());
            telemetry.addData("Time of Flight (sec)", timeOfFlightSec);
            telemetry.addLine("");
        }
    }

    public void fire() {
        gateServoController.setPosition(GATE_SERVO_OPEN, GATE_SERVO_TIME_MS, GATE_SERVO_CLOSED);
        transferServoController.setPosition(TRANSFER_SERVO_UP, TRANSFER_SERVO_TIME_MS, TRANSFER_SERVO_DOWN);

        laucnhCooldownTimer.resetTimer();
    }

    public void startAllSubsystems() {
        setFlywheelOn(true);
        setIntakeOn(true);
        setAutoAimTurntable(true);
    }

    public void setIntakeOn(boolean isIntakeOn) {
        this.isIntakeOn = isIntakeOn;
    }

    public boolean isIntakeOn() {
        return isIntakeOn;
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

    public boolean isFlywheelOn() {
        return isFlywheelOn;
    }

    public void setFlywheelOn(boolean flywheelOn) {
        isFlywheelOn = flywheelOn;
    }

    public boolean isReadyToShoot() {
        return isReadyToShoot;
    }

    public LimelightController getLimelightHandler() {
        return limelightController;
    }

    public StateMachine getAutoFire() {
        return autoFire;
    }

    public Turret getTurret() {
        return turret;
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