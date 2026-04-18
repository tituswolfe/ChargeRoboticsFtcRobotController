package org.firstinspires.ftc.teamcode.robots.season.decode.jetfire;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.hardware.controllers.digital.GoBildaLaserDistanceSensorDigital;
import org.firstinspires.ftc.teamcode.hardware.controllers.lights.GoBildaPrismController;
import org.firstinspires.ftc.teamcode.hardware.controllers.lights.Prism.GoBildaPrismDriver;
import org.firstinspires.ftc.teamcode.hardware.controllers.motor.ThrottleMotorController;
import org.firstinspires.ftc.teamcode.hardware.controllers.motor.TurntablePIDFMotorController;
import org.firstinspires.ftc.teamcode.hardware.controllers.servo.AngleServoController;
import org.firstinspires.ftc.teamcode.hardware.controllers.servo.PositionServoController;
import org.firstinspires.ftc.teamcode.hardware.controllers.servo.RGBIndicatorLightController;
import org.firstinspires.ftc.teamcode.hardware.controllers.motor.DualPIDFMotorVelocityController;
import org.firstinspires.ftc.teamcode.hardware.controllers.subsystems.Turret;
import org.firstinspires.ftc.teamcode.hardware.drivetrain.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.hardware.vision.LimelightController;
import org.firstinspires.ftc.teamcode.robots.base.RobotBase;
import org.firstinspires.ftc.teamcode.robots.base.opmodes.OpModeBase;
import org.firstinspires.ftc.teamcode.util.actionsequence.Action;
import org.firstinspires.ftc.teamcode.util.actionsequence.ActionSequence;
import org.firstinspires.ftc.teamcode.util.actionsequence.InstantAction;
import org.firstinspires.ftc.teamcode.util.actionsequence.Wait;
import org.firstinspires.ftc.teamcode.util.info.HardwareInfo;
import org.firstinspires.ftc.teamcode.util.math.Conversion;
import org.firstinspires.ftc.teamcode.util.math.MathUtil;
import org.firstinspires.ftc.teamcode.util.math.RollingAverage;

import static org.firstinspires.ftc.teamcode.robots.base.StaticData.allianceColor;
import static org.firstinspires.ftc.teamcode.robots.season.decode.jetfire.JetFireConstants.*;

import java.util.Optional;
import java.util.concurrent.TimeUnit;

@Configurable
public class JetfireRobot extends RobotBase {
    private Turret<DualPIDFMotorVelocityController> turret;

    ThrottleMotorController intakeController;
    private GoBildaLaserDistanceSensorDigital intakeSensor;

    PositionServoController gateServoController;

    private RGBIndicatorLightController indicatorLightController;
    private GoBildaPrismController prismController;

    private LimelightController limelightController;

    private boolean autoAimTurntable = false;
    private boolean isFlywheelOn = false;
    private boolean isIntakeOn = false;
    private boolean reverseIntake = false;

    private boolean useMuzzleFlash = false;

    public double closeTurntableOffsetDeg = 0; // getter setter
    public double farTurntableOffsetDeg = 0;

    private final static Pose HUMAN_PLAYER_ZONE_RESET_BLUE = new Pose(134.9, 12.2, Math.toRadians(0));
    private Pose humanPlayerReset;

    private static final double GOAL_AIM_OFFSET = 5;
    private final static Pose TARGET_GOAL_BLUE = new Pose(0 + GOAL_AIM_OFFSET, 144 - GOAL_AIM_OFFSET, 0);
    public static Pose targetGoal;

    // TUNING & Telemetry
    public static boolean DISPLAY_TELEMETRY = false;
    public static boolean TUNING = false;
    public static double TUNING_TARGET_FLYWHEEL_VELOCITY = 0;
    public static double TUNING_HOOD_ANGLE = 16;

    // RUNTIME
    private boolean isReadyToShoot = false;
    private boolean isInFarZone = false;
    private boolean isFlywheelReady = false;

    // Intake Sensor
    Timer artifactDetectedTimer = new Timer();
    private boolean wasArtifactDetected = false;

    // Action Sequences
    private final Action[] rapidFireActions = new Action[] {
            new InstantAction(() -> {
                gateServoController.setPosition(GATE_SERVO_OPEN);
                intakeController.setMotorEngaged(true);
                intakeController.setTargetPower(-0.75);
            }),
            new Wait(1),
            new InstantAction(() -> intakeController.setTargetPower(1.0)),
            new Wait(INTAKE_RAPID_FIRE_DURATION_MS),
            new InstantAction(() -> gateServoController.setPosition(GATE_SERVO_CLOSED))
    };
    private final ActionSequence rapidFireActionSequence = new ActionSequence(rapidFireActions);

    RollingAverage velocitySmoothing = new RollingAverage(3);

    @Override
    public void init(HardwareMap hardwareMap, Pose startPose, OpModeBase.AllianceColor allianceColor) {
        super.init(hardwareMap, startPose, allianceColor);

        if (allianceColor.equals(OpModeBase.AllianceColor.BLUE)) {
            prismController.setBaseArtboard(GoBildaPrismDriver.Artboard.ARTBOARD_1);
            targetGoal = TARGET_GOAL_BLUE;
            humanPlayerReset = HUMAN_PLAYER_ZONE_RESET_BLUE;

            closeTurntableOffsetDeg = CLOSE_ZONE_TURNTABLE_START_OFFSET_BLUE;
            farTurntableOffsetDeg = FAR_ZONE_TURNTABLE_START_OFFSET_BLUE;
        } else {
            prismController.setBaseArtboard(GoBildaPrismDriver.Artboard.ARTBOARD_2);
            targetGoal = TARGET_GOAL_BLUE.mirror();
            humanPlayerReset = HUMAN_PLAYER_ZONE_RESET_BLUE.mirror();

            closeTurntableOffsetDeg = CLOSE_ZONE_TURNTABLE_START_OFFSET_RED;
            farTurntableOffsetDeg = FAR_ZONE_TURNTABLE_START_OFFSET_RED;
        }
    }

    @Override
    public void stop() {
        prismController.setBaseArtboard(GoBildaPrismDriver.Artboard.ARTBOARD_0);
        super.stop();
    }

    @Override
    public void initHardware(HardwareMap hardwareMap) {
        DcMotorEx topFlywheelMotor = hardwareMap.get(DcMotorEx.class, TOP_FLYWHEEL_MOTOR_DEVICE_NAME);
        topFlywheelMotor.setDirection(TOP_FLYWHEEL_MOTOR_DIRECTION);

        DcMotorEx bottomFlywheelMotor = hardwareMap.get(DcMotorEx.class, BOTTOM_FLYWHEEL_MOTOR_DEVICE_NAME);
        bottomFlywheelMotor.setDirection(BOTTOM_FLYWHEEL_MOTOR_DIRECTION);

        DualPIDFMotorVelocityController flywheelController = new DualPIDFMotorVelocityController(
                bottomFlywheelMotor,
                topFlywheelMotor,
                FLYWHEEL_NAME,
                HardwareInfo.GOBILDA_5203_YELLOW_JACKET_MOTOR_6000_RPM,
                FLYWHEEL_GEAR_RATIO,
                FLYWHEEL_MAX_POWER,
                FLYWHEEL_PIDF_COEFFICIENTS

        );

        DcMotorEx turntableMotor = hardwareMap.get(DcMotorEx.class, TURNTABLE_MOTOR_DEVICE_NAME);
        double turretStartHeading = Optional.of(JetfireStaticData.lastTurretHeading).orElse(0.0);

        TurntablePIDFMotorController turntableController = new TurntablePIDFMotorController(
                turntableMotor,
                TURNTABLE_NAME,
                HardwareInfo.GOBILDA_5203_YELLOW_JACKET_MOTOR_435_RPM,
                TURNTABLE_TOTAL_GEAR_RATIO,
                TURNTABLE_MAX_POWER,
                TURNTABLE_PIDF_COEFFICIENTS,
                TURNTABLE_MIN_HARD_STOP,
                TURNTABLE_MAX_HARD_STOP
        );
        turntableController.setInitialAngle(turretStartHeading);

        Servo hoodServo = hardwareMap.get(Servo.class, HOOD_SERVO_DEVICE_NAME);
        hoodServo.setDirection(HOOD_SERVO_DIRECTION);
        hoodServo.setPosition(0.5);

        AngleServoController hoodController = new AngleServoController(
                hoodServo,
                HOOD_NAME,
                HOOD_TOTAL_ROTATION,
                HOOD_TOTAL_GEAR_RATIO,
                MIN_HOOD_ANGLE,
                MAX_HOOD_ANGLE
        );

        turret = new Turret<>(flywheelController, turntableController, hoodController);

        DcMotorEx intake = hardwareMap.get(DcMotorEx.class, INTAKE_MOTOR_DEVICE_NAME);
        intake.setDirection(INTAKE_DIRECTION);
        intakeController = new ThrottleMotorController(
                intake,
                INTAKE_NAME,
                HardwareInfo.GOBILDA_5203_YELLOW_JACKET_MOTOR_1150_RPM,
                INTAKE_GEAR_RATIO,
                INTAKE_MAX_POWER
        );

        Servo gateServo = hardwareMap.get(Servo.class, "gate");
        gateServoController = new PositionServoController(
                gateServo,
                "gate",
                300, // TODO: Only controller?
                1
        );

        DigitalChannel laserDigitalInput = hardwareMap.get(DigitalChannel.class, "laser-digital-input");
        intakeSensor = new GoBildaLaserDistanceSensorDigital(laserDigitalInput);

        Limelight3A limelight3A = hardwareMap.get(Limelight3A.class, "limelight");
        limelightController = new LimelightController(limelight3A, "Limelight");
        limelightController.init(LIMELIGHT_LOCALIZATION_PIPELINE);

        Servo indicator = hardwareMap.get(Servo.class, "indicator");
        indicatorLightController = new RGBIndicatorLightController(indicator, "Indicator");
        indicatorLightController.setBaseColor(allianceColor == OpModeBase.AllianceColor.RED ? RGBIndicatorLightController.Color.RED : RGBIndicatorLightController.Color.BLUE);
        indicatorLightController.update(0);

        GoBildaPrismDriver prism = hardwareMap.get(GoBildaPrismDriver.class,"prism");
        prismController = new GoBildaPrismController(prism, "Prism");

        gateServoController.start(GATE_SERVO_CLOSED);
        intakeController.start();
        turret.turntableController().start();
        turret.flywheelController().start();
        turret.hoodServoController().start(0); // TODO: Get from data table
    }

    @Override
    public void startConfiguration() {
//        gateServoController.start(GATE_SERVO_CLOSED);
//        intakeController.start();
//        turret.turntableController().start();
//        turret.flywheelController().start();
//        turret.hoodServoController().start(0); // TODO: Get from data table
    }

    @Override
    public void update(long deltaTimeNs, TelemetryManager telemetry) {
        Pose currentPose = follower.getPose();
        Vector velocity = follower.getVelocity();

        double velocityMagnitude = follower.getVelocity().getMagnitude();
        double angularVelocity = follower.getAngularVelocity();

        velocitySmoothing.update(velocityMagnitude);
        velocity = velocity.times(velocityMagnitude / velocitySmoothing.getAverage()); // ??

        double turntableHeading = turret.turntableController().getHeading();
        double flywheelError = turret.flywheelController().getError();

        // Lead Computing
        double launchDelaySec = Conversion.millsToSec(INTAKE_TRANSFER_DELAY_MS);
        Pose predictedFuturePose = MathUtil.predictFuturePose(currentPose, velocity, launchDelaySec);
        double distanceFromGoalAtFuturePose = predictedFuturePose.distanceFrom(targetGoal);

        double timeOfFlightSec = Conversion.millsToSec((int) TIME_OF_FLIGHT_BY_DISTANCE.interpolate(distanceFromGoalAtFuturePose));
        Vector virtualGoalVelocity = velocity.times(-1);
        Pose virtualGoal = MathUtil.predictFuturePose(targetGoal, virtualGoalVelocity, timeOfFlightSec);

        double virtualDistanceFromGoal = predictedFuturePose.distanceFrom(virtualGoal);
        double virtualGoalHeading = MathUtil.bearingTo(predictedFuturePose, virtualGoal);

        // CONDITIONS
        isInFarZone = currentPose.getY() < FAR_ZONE_Y_THRESHOLD;
        isFlywheelReady = flywheelError < FLYWHEEL_VELOCITY_MARGIN_RPM && isFlywheelOn;

        //double turntableErrorDeg = Math.toDegrees(turret.turntableController().getError());
        //boolean isTurntableReady = Math.abs(turntableErrorDeg) < TURNTABLE_HEADING_MARGIN_DEG && autoAimTurntable;
        double goalHeadingError = Math.abs(AngleUnit.normalizeRadians(virtualGoalHeading - currentPose.getHeading()));
        boolean isTurntableInRange =  goalHeadingError < TURNTABLE_MAX_HARD_STOP;

        boolean isArtifactDetected = intakeSensor.isObjectDetected();
        if (isArtifactDetected && !wasArtifactDetected) {
            artifactDetectedTimer.resetTimer();
        }

        // intake
        boolean pauseIntake = false;
        boolean indicateIntakeFull = false;
        if (isArtifactDetected) {
            pauseIntake = artifactDetectedTimer.getElapsedTime() > INTAKE_TIMEOUT_MS;
            indicateIntakeFull = artifactDetectedTimer.getElapsedTime() > INDICATE_FULL_INTAKE_MS;
        }

        wasArtifactDetected = isArtifactDetected;

        isReadyToShoot = !getRapidFireActionSequence().isRunning() && isTurntableInRange;

        // HARDWARE VARIABLES
        double flywheelSpeed = FLYWHEEL_VELOCITY_BY_DISTANCE.interpolate(virtualDistanceFromGoal);

        double turntableZoneOffsetDeg = isInFarZone ? farTurntableOffsetDeg : closeTurntableOffsetDeg;
        double targetTurntableHeading = AngleUnit.normalizeRadians(virtualGoalHeading - currentPose.getHeading()) + Math.toRadians(turntableZoneOffsetDeg);

        double interpolatedHoodAngleDeg = HOOD_ANGLE_BY_DISTANCE.interpolate(virtualDistanceFromGoal);

        double hoodCompensation = 0;
        if (flywheelError > FLYWHEEL_ERROR_COMPENSATION_THRESHOLD && interpolatedHoodAngleDeg > HOOD_COMPENSATION_FLOOR_DEG && isInFarZone) {
            double maxCompensation = interpolatedHoodAngleDeg - HOOD_COMPENSATION_FLOOR_DEG;
            hoodCompensation = Range.clip(Math.abs(flywheelError) * REGRESSION_COMPENSATION_RATIO, 0, maxCompensation);
        }

       // double interpolatedHoodAngleDeg = HOOD_ANGLE_BY_DISTANCE.interpolate(virtualDistanceFromGoal);
//
//        if (flywheelError > FLYWHEEL_ERROR_COMPENSATION_THRESHOLD) {
//            double rawComp = flywheelError * REGRESSION_COMPENSATION_RATIO;
//
//            if (interpolatedHoodAngleDeg > 45.0) {
//                double maxPossibleComp = interpolatedHoodAngleDeg - 45.0;
//                double finalComp = Math.min(rawComp, maxPossibleComp);
//                interpolatedHoodAngleDeg -= finalComp;
//            } else {
//                double maxPossibleComp = 45.0 - interpolatedHoodAngleDeg;
//                double finalComp = Math.min(rawComp, maxPossibleComp);
//                interpolatedHoodAngleDeg += finalComp;
//            }
//        }

        // compensate the other way

        double hoodAngle = Math.toRadians(interpolatedHoodAngleDeg - hoodCompensation);

        if (!rapidFireActionSequence.isRunning()) {
            if (reverseIntake) {
                intakeController.setMotorEngaged(true);
                intakeController.setTargetPower(REVERSE_INTAKE_POWER);
            } else if (pauseIntake) {
                intakeController.setMotorEngaged(false);
            } else if (isIntakeOn) {
                intakeController.setMotorEngaged(true);
                intakeController.setTargetPower(INTAKE_POWER);
            } else {
                intakeController.setMotorEngaged(false);
            }
        }

        if (indicateIntakeFull) {
            prismController.indicate(GoBildaPrismDriver.Artboard.ARTBOARD_3, 50);
        }

        RGBIndicatorLightController.Color indicatorColor;
        if (isReadyToShoot) {
            indicatorColor = RGBIndicatorLightController.Color.GREEN;
        } else if (!isTurntableInRange) {
            indicatorColor = RGBIndicatorLightController.Color.ORANGE;
        } else {
            indicatorColor = RGBIndicatorLightController.Color.RED;
        }

        // UPDATE HARDWARE CONTROLLERS
        super.update(deltaTimeNs, telemetry);

        turret.flywheelController().setMotorEngaged(isFlywheelOn);
        turret.turntableController().setMotorEngaged(autoAimTurntable);
        turret.turntableController().updateRobotHeadingVelocity(angularVelocity);

        if (TUNING) {
            turret.update(TUNING_TARGET_FLYWHEEL_VELOCITY, targetTurntableHeading, TUNING_HOOD_ANGLE, deltaTimeNs);

            turret.flywheelController().setPIDFCoefficients(FLYWHEEL_PIDF_COEFFICIENTS);
            turret.turntableController().setPIDFCoefficients(TURNTABLE_PIDF_COEFFICIENTS);
        } else {
            turret.update(flywheelSpeed, targetTurntableHeading, hoodAngle, deltaTimeNs);
        }

        limelightController.updateRobotHeading(Math.toDegrees(currentPose.getHeading()));
        limelightController.update(deltaTimeNs);

        indicatorLightController.setBaseColor(indicatorColor);
        indicatorLightController.update(deltaTimeNs);

        //gateServoController.update();

        intakeController.update(deltaTimeNs);
        prismController.update(0);

        intakeSensor.update();

        // UPDATE ACTION SEQUENCES
        rapidFireActionSequence.update();

        // UPDATE STATIC VARS
        JetfireStaticData.lastTurretHeading = turntableHeading;

        if (DISPLAY_TELEMETRY) {
            double distanceFromGoal = currentPose.distanceFrom(targetGoal);

            // Variable Telemetry
            telemetry.addLine("- MISC. VARS -");
            telemetry.addData("Rapid Fire Status", rapidFireActionSequence.isRunning() ? 1 : 0);
            telemetry.addLine("");

            telemetry.addLine("- OFFSETS -");
            telemetry.addData("isInFarZone", isInFarZone);
            telemetry.addData("Close Turntable Offset", closeTurntableOffsetDeg);
            telemetry.addData("Far Turntable Offset", farTurntableOffsetDeg);
            telemetry.addLine("");

            telemetry.addLine("- GOAL -");
            telemetry.addData("Distance From Goal", distanceFromGoal);
            telemetry.addData("Heading to Goal", Math.toDegrees(MathUtil.bearingTo(currentPose, targetGoal)));
            telemetry.addLine("");

            telemetry.addLine("- TURRET -");
            telemetry.addData("Flywheel Velocity Error", flywheelError);
            telemetry.addData("Interpolated Hood Angle (deg)",interpolatedHoodAngleDeg);
            telemetry.addData("Hood Compensation (deg)", hoodCompensation);
            telemetry.addLine("");

            telemetry.addLine("- LEAD COMPUTING - ");
            telemetry.addData("Future Pose X", predictedFuturePose.getX());
            telemetry.addData("Future Pose Y", predictedFuturePose.getY());
            telemetry.addData("Time of Flight (sec)", timeOfFlightSec);
            telemetry.addData("Heading to Virtual Goal", Math.toDegrees(virtualGoalHeading));
            telemetry.addLine("");

            telemetry.addLine("- CONDITIONS -");
            telemetry.addData("isReadyToShoot", isReadyToShoot);
            telemetry.addData("isFlywheelReady", isFlywheelReady);
            telemetry.addLine("");

            // Hardware Telemetry
            limelightController.updateTelemetry(telemetry);
            turret.turntableController().updateTelemetry(telemetry);
            turret.flywheelController().updateTelemetry(telemetry);
            turret.hoodServoController().updateTelemetry(telemetry);

            telemetry.addLine("");
        }
    }

    public void adjustActiveTurntableZoneOffset(double offsetDeg) {
        if (isInFarZone) {
            farTurntableOffsetDeg += offsetDeg;
        } else {
            closeTurntableOffsetDeg += offsetDeg;
        }
    }

    public void zeroTurntable() {
        turret.turntableController().setInitialAngle(0);

        if (allianceColor.equals(OpModeBase.AllianceColor.BLUE)) {
            closeTurntableOffsetDeg = JetFireConstants.CLOSE_ZONE_TURNTABLE_START_OFFSET_BLUE;
            farTurntableOffsetDeg = JetFireConstants.FAR_ZONE_TURNTABLE_START_OFFSET_BLUE;
        } else {
            closeTurntableOffsetDeg = JetFireConstants.CLOSE_ZONE_TURNTABLE_START_OFFSET_RED;
            farTurntableOffsetDeg = JetFireConstants.FAR_ZONE_TURNTABLE_START_OFFSET_RED;
        }
        indicatorLightController.indicate(RGBIndicatorLightController.Color.VIOLET);
    }

    public void fire() {
        if (!rapidFireActionSequence.isRunning()) {
            rapidFireActionSequence.start();
            if (useMuzzleFlash) {
                prismController.indicate(GoBildaPrismDriver.Artboard.ARTBOARD_4, MUZZLE_FLASH_DURATION_MS);
                //indicatorLightController.indicate(RGBIndicatorLightController.Color.ORANGE, MUZZLE_FLASH_DURATION_MS);
            }
        }
    }

    public void toggleMuzzleFlash() {
        useMuzzleFlash = !useMuzzleFlash;
    }

    public void humanPlayerPoseReset() {
        follower.setPose(humanPlayerReset);
        indicatorLightController.indicate(RGBIndicatorLightController.Color.VIOLET);
    }

    public void toggleSubsystems(boolean on) {
        setFlywheelOn(on);
        setIntakeOn(on);
        setAutoAimTurntable(on);
    }

    public GoBildaPrismController getPrismController() {
        return prismController;
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

    public RGBIndicatorLightController getIndicatorLightController() {
        return indicatorLightController;
    }

    public ActionSequence getRapidFireActionSequence() {
        return rapidFireActionSequence;
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