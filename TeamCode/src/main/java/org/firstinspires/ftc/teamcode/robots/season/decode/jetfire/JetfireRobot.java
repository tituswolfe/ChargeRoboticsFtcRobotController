package org.firstinspires.ftc.teamcode.robots.season.decode.jetfire;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.control.PIDFCoefficients;
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

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.hardware.controllers.digital.GoBildaLaserDistanceSensorDigital;
import org.firstinspires.ftc.teamcode.hardware.controllers.lights.Prism.Color;
import org.firstinspires.ftc.teamcode.hardware.controllers.lights.Prism.GoBildaPrismDriver;
import org.firstinspires.ftc.teamcode.hardware.controllers.lights.Prism.PrismAnimations;
import org.firstinspires.ftc.teamcode.hardware.controllers.motor.TurntablePIDFMotorController;
import org.firstinspires.ftc.teamcode.hardware.controllers.motor.VelocityPIDFMotorController;
import org.firstinspires.ftc.teamcode.hardware.controllers.servo.AngleServoController;
import org.firstinspires.ftc.teamcode.hardware.controllers.servo.RGBIndicatorLightController;
import org.firstinspires.ftc.teamcode.hardware.controllers.servo.ServoTimerController;
import org.firstinspires.ftc.teamcode.hardware.controllers.motor.DualPIDFMotorVelocityController;
import org.firstinspires.ftc.teamcode.hardware.controllers.subsystems.Turret;
import org.firstinspires.ftc.teamcode.hardware.drivetrain.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.hardware.vision.LimelightController;
import org.firstinspires.ftc.teamcode.robots.base.RobotBase;
import org.firstinspires.ftc.teamcode.robots.base.opmodes.OpModeBase;
import org.firstinspires.ftc.teamcode.util.PoseHistory;
import org.firstinspires.ftc.teamcode.util.info.HardwareInfo;
import org.firstinspires.ftc.teamcode.util.math.Angle;
import org.firstinspires.ftc.teamcode.util.math.MathUtil;

import static org.firstinspires.ftc.teamcode.robots.base.StaticData.allianceColor;
import static org.firstinspires.ftc.teamcode.robots.season.decode.jetfire.JetFireConstants.*;

import java.util.Optional;

@Configurable
public class JetfireRobot extends RobotBase {
    private Turret<DualPIDFMotorVelocityController> turret;
    DcMotorEx intake;

    private ServoTimerController transferServoController;
    private ServoTimerController gateServoController;

    private GoBildaLaserDistanceSensorDigital chamberSensor;

    private RGBIndicatorLightController indicatorLightController;
    private GoBildaPrismDriver prism;

    private LimelightController limelightController;

    private boolean autoAimTurntable = false;
    private boolean isFlywheelOn = false;
    private boolean isIntakeOn = false;
    private boolean reverseIntake = false;

    // Positive offset is right
    // Negative offset is left
    private static double CLOSE_ZONE_TURNTABLE_START_OFFSET_BLUE = 2;
    private static double FAR_ZONE_TURNTABLE_START_OFFSET_BLUE = -4;
    public static double CLOSE_ZONE_TURNTABLE_OFFSET_DEG;
    public static double FAR_ZONE_TURNTABLE_OFFSET_DEG;

    Timer laucnhCooldownTimer = new Timer();

    PoseHistory poseHistory = new PoseHistory(5);

    public static Pose humanPlayerReset = new Pose(64.2, -58.08, Math.toRadians(-90));

    // TODO: LUTs

    // TUNING
    public static boolean TUNING = false;
    public static double FLYWHEEL_TARGET_VELOCITY = 0;
    public static double HOOD_ANGLE = 16;
    public static double TURNTABLE_OFFSET_DEG = 0;

    // RUNTIME
    public static Pose targetGoal = new Pose(-67, 67);
    private boolean isReadyToShoot = false;
    private boolean isInFarZone = false;
    private boolean isFlywheelReady = false;
    private boolean isArtifactLoaded = false;

//    private final StateMachine autoFire = new StateMachine();

    @Override
    public void init(HardwareMap hardwareMap, Pose startPose, OpModeBase.AllianceColor allianceColor) {
        super.init(hardwareMap, startPose, allianceColor);

        if (allianceColor.equals(OpModeBase.AllianceColor.BLUE)) {
            // .mirror() only works using Pedro's coordinate system
            targetGoal = targetGoal.withY(-67);
            humanPlayerReset = humanPlayerReset.withY(58.08).withHeading(Math.toRadians(-90));

            CLOSE_ZONE_TURNTABLE_OFFSET_DEG = CLOSE_ZONE_TURNTABLE_START_OFFSET_BLUE;
            FAR_ZONE_TURNTABLE_OFFSET_DEG = FAR_ZONE_TURNTABLE_START_OFFSET_BLUE;
        }
    }

    @Override
    public void initHardware(HardwareMap hardwareMap) {
        DcMotorEx leftFlywheelMotor = hardwareMap.get(DcMotorEx.class, LEFT_FLYWHEEL_MOTOR_DEVICE_NAME);
        leftFlywheelMotor.setDirection(LEFT_FLYWHEEL_MOTOR_DIRECTION);

        DcMotorEx rightFlywheelMotor = hardwareMap.get(DcMotorEx.class, RIGHT_FLYWHEEL_MOTOR_DEVICE_NAME);
        rightFlywheelMotor.setDirection(RIGHT_FLYWHEEL_MOTOR_DIRECTION);

        DualPIDFMotorVelocityController flywheelController = new DualPIDFMotorVelocityController(
                leftFlywheelMotor,
                rightFlywheelMotor,
                FLYWHEEL_NAME,
                FLYWHEEL_PIDF_COEFFICIENTS,
                HardwareInfo.GOBILDA_5203_YELLOW_JACKET_MOTOR_6000_RPM.ENCODER_RESOLUTION_PPR,
                FLYWHEEL_GEAR_RATIO,
                FLYWHEEL_MAX_POWER
        );


        DcMotorEx turntableMotor = hardwareMap.get(DcMotorEx.class, TURNTABLE_MOTOR_DEVICE_NAME);
        double turretStartHeading = Optional.of(JetfireStaticData.lastTurretHeading).orElse(0.0);

        TurntablePIDFMotorController turntableController = new TurntablePIDFMotorController(
                turntableMotor,
                TURNTABLE_NAME,
                TURNTABLE_PIDF_COEFFICIENTS,
                HardwareInfo.GOBILDA_5203_YELLOW_JACKET_MOTOR_435_RPM.ENCODER_RESOLUTION_PPR,
                TURNTABLE_TOTAL_GEAR_RATIO,
                TURNTABLE_MAX_POWER,
                TURNTABLE_MIN_HARD_STOP,
                TURNTABLE_MAX_HARD_STOP,
                REVERSE_POWER,
                turretStartHeading
        );

        Servo hoodServo = hardwareMap.get(Servo.class, HOOD_SERVO_DEVICE_NAME);
        hoodServo.setDirection(HOOD_SERVO_DIRECTION);

        AngleServoController hoodController = new AngleServoController(
                hoodServo,
                HOOD_NAME,
                HOOD_TOTAL_ROTATION,
                HOOD_TOTAL_GEAR_RATIO,
                MIN_HOOD_ANGLE,
                MAX_HOOD_ANGLE
        );

        turret = new Turret<>(flywheelController, turntableController, hoodController);

        intake = hardwareMap.get(DcMotorEx.class, INTAKE_MOTOR_DEVICE_NAME);
        intake.setDirection(INTAKE_DIRECTION);
        intake.setCurrentAlert(INTAKE_CURRENT_THRESHOLD_AMPS, CurrentUnit.AMPS);

        Servo gateServo = hardwareMap.get(Servo.class, "gate");
        gateServoController = new ServoTimerController(gateServo, GATE_SERVO_CLOSED);

        Servo transferServo = hardwareMap.get(Servo.class, "transfer");
        transferServoController = new ServoTimerController(transferServo, TRANSFER_SERVO_DOWN);

        DigitalChannel laserDigitalInput = hardwareMap.get(DigitalChannel.class, "laser-digital-input");
        chamberSensor = new GoBildaLaserDistanceSensorDigital(laserDigitalInput);

        Limelight3A limelight3A = hardwareMap.get(Limelight3A.class, "limelight");
        limelightController = new LimelightController(limelight3A, "Limelight");
        limelightController.init(LIMELIGHT_LOCALIZATION_PIPELINE);

        Servo indicator = hardwareMap.get(Servo.class, "indicator");
        indicatorLightController = new RGBIndicatorLightController(indicator, "Indicator");
        indicatorLightController.setBaseColor(allianceColor == OpModeBase.AllianceColor.RED ? RGBIndicatorLightController.Color.RED : RGBIndicatorLightController.Color.BLUE);
        indicatorLightController.update();

        prism = hardwareMap.get(GoBildaPrismDriver.class,"prism");
        PrismAnimations.DroidScan droidScan = new PrismAnimations.DroidScan();
        droidScan.setPrimaryColor(allianceColor == OpModeBase.AllianceColor.RED ? Color.RED : Color.BLUE);
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
        Pose currentPose = follower.getPose();
        Vector velocity = follower.getVelocity();
        double velocityMagnitude = velocity.getMagnitude();
        double angularVelocity = follower.getAngularVelocity();

        double turntableHeading = turret.turntableController().getHeading();
        double flywheelError = turret.flywheelController().getError();

        // Lead Computing
        long totalLaunchDelayMills = LEAD_COMPUTING_TRANSFER_DELAY_MS; // deltaTimeMS
        double launchDelaySec = totalLaunchDelayMills / 1000.0;

        Pose predictedFuturePose = MathUtil.predictFuturePose(currentPose, velocity, launchDelaySec);
        double distanceFromGoalAtFuturePose = predictedFuturePose.distanceFrom(targetGoal);

        long timeOfFlightMills = (long) TIME_OF_FLIGHT_BY_DISTANCE.interpolate(distanceFromGoalAtFuturePose);
        double timeOfFlightSec = timeOfFlightMills / 1000.0;

        Vector virtualGoalVelocity = velocity.times(-1);
        Pose virtualGoal = MathUtil.predictFuturePose(targetGoal, virtualGoalVelocity, timeOfFlightSec);

        double virtualDistanceFromGoal = predictedFuturePose.distanceFrom(virtualGoal);
        double virtualGoalHeading = MathUtil.bearingTo(predictedFuturePose, virtualGoal);

        // CONDITIONS

        isInFarZone = currentPose.getX() > FAR_ZONE_X_THRESHOLD;
        isFlywheelReady = Math.abs(flywheelError) <= FLYWHEEL_VELOCITY_MARGIN_RPM && isFlywheelOn;

        double turntableErrorDeg = Math.toDegrees(turret.turntableController().getError());
        boolean isTurntableReady = Math.abs(turntableErrorDeg) < TURNTABLE_HEADING_MARGIN_DEG && autoAimTurntable;

        double goalHeadingError = Math.abs(AngleUnit.normalizeRadians(virtualGoalHeading - currentPose.getHeading()));
        boolean isTurntableInRange =  goalHeadingError < Math.toRadians(135);

        isArtifactLoaded = chamberSensor.isObjectDetected();

        boolean isCooldownOver = laucnhCooldownTimer.getElapsedTime() > LAUNCH_COOLDOWN_MS;
        boolean isIntakeCooldownOver = laucnhCooldownTimer.getElapsedTime() > INTAKE_COOLDOWN_MS;

        isReadyToShoot = isCooldownOver && isTurntableInRange;


        // HARDWARE VARIABLES
        double flywheelSpeed = FLYWHEEL_VELOCITY_BY_DISTANCE.interpolate(virtualDistanceFromGoal);

        double turntableZoneOffsetDeg = isInFarZone ? FAR_ZONE_TURNTABLE_OFFSET_DEG : CLOSE_ZONE_TURNTABLE_OFFSET_DEG;
        double targetTurntableHeading = AngleUnit.normalizeRadians(virtualGoalHeading - currentPose.getHeading()) + Math.toRadians(turntableZoneOffsetDeg);

        double interpolatedHoodAngleDeg = HOOD_ANGLE_BY_DISTANCE.interpolate(virtualDistanceFromGoal);
        double hoodRegression = flywheelError * HOOD_REGRESSION_RATIO;
        Angle hoodAngle = new Angle(interpolatedHoodAngleDeg - hoodRegression, Angle.AngleUnit.DEGREES);

        double intakePower = 0.0;
        if (isIntakeOn && isIntakeCooldownOver && !intake.isOverCurrent()) {
            intakePower = reverseIntake ? REVERSE_INTAKE_POWER : INTAKE_POWER;
        }
        intake.setPower(intakePower);

        RGBIndicatorLightController.Color indicatorColor;
        if (isReadyToShoot) {
            indicatorColor = RGBIndicatorLightController.Color.GREEN;
        } else if (!isTurntableInRange) {
            indicatorColor = RGBIndicatorLightController.Color.ORANGE;
        } else {
            indicatorColor = RGBIndicatorLightController.Color.RED;
        }

        // - UPDATE HARDWARE -
        super.update(deltaTimeMs, telemetry);

        turret.flywheelController().setMotorEngaged(isFlywheelOn);
        turret.turntableController().setMotorEngaged(autoAimTurntable);

        turret.update(flywheelSpeed, targetTurntableHeading, hoodAngle);
        turret.turntableController().setDriveTrainHeadingVelocity(angularVelocity);

        chamberSensor.update();

        limelightController.updateRobotHeading(Math.toDegrees(currentPose.getHeading()));
        limelightController.update();

        transferServoController.update();
        gateServoController.update();

        indicatorLightController.setBaseColor(indicatorColor);
        indicatorLightController.update();

        // UPDATE STATIC
        JetfireStaticData.lastTurretHeading = turntableHeading;

        // - TELEMETRY -

        telemetry.addLine("- VARS -");
        telemetry.addData("flywheel error", flywheelError);
        telemetry.addData("hood regression (deg)", hoodRegression);
        telemetry.addLine("");

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
        telemetry.addLine("");

        limelightController.updateTelemetry(telemetry);
        turret.turntableController().updateTelemetry(telemetry);
        turret.flywheelController().updateTelemetry(telemetry);
        turret.hoodServoController().updateTelemetry(telemetry);

        telemetry.addData("Interpolated Hood Angle (Deg)", interpolatedHoodAngleDeg);
        telemetry.addData("Additive Linear Offset Regression (Deg)", hoodRegression);
        telemetry.addLine("");
        telemetry.addLine("- LEAD COMPUTING & LUTs -");
        telemetry.addData("Future Pose X", predictedFuturePose.getX());
        telemetry.addData("Future Pose Y", predictedFuturePose.getY());
        telemetry.addData("Time of Flight (sec)", timeOfFlightSec);
        telemetry.addLine("");

        double distanceFromGoal = currentPose.distanceFrom(targetGoal);

        telemetry.addLine("");
        telemetry.addLine("- TUNING DATA -");
        telemetry.addData("Distance From Goal", distanceFromGoal);

        if (TUNING) {
            if (!turret.flywheelController().getPidfController().getCoefficients().equals(FLYWHEEL_PIDF_COEFFICIENTS)) turret.flywheelController().getPidfController().setCoefficients(FLYWHEEL_PIDF_COEFFICIENTS);
            if (!turret.turntableController().getPidfController().getCoefficients().equals(TURNTABLE_PIDF_COEFFICIENTS)) turret.turntableController().getPidfController().setCoefficients(TURNTABLE_PIDF_COEFFICIENTS);

            telemetry.addLine("");
        }
    }

    public void fire() {
        gateServoController.setPosition(GATE_SERVO_OPEN, GATE_SERVO_TIME_MS, GATE_SERVO_CLOSED);
        transferServoController.setPosition(TRANSFER_SERVO_UP, TRANSFER_SERVO_TIME_MS, TRANSFER_SERVO_DOWN);

        laucnhCooldownTimer.resetTimer();
    }

    public void toggleSubsystems(boolean on) {
        setFlywheelOn(on);
        setIntakeOn(on);
        setAutoAimTurntable(on);
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

//    public StateMachine getAutoFire() {
//        return autoFire;
//    }

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

    public RGBIndicatorLightController getIndicatorLightController() {
        return indicatorLightController;
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