package org.firstinspires.ftc.teamcode.robots.season.decode.jetfire;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.hardware.controllers.motor.AngleMotorController;
import org.firstinspires.ftc.teamcode.hardware.controllers.motor.VelocityMotorController;
import org.firstinspires.ftc.teamcode.hardware.controllers.servo.AngleServoController;
import org.firstinspires.ftc.teamcode.hardware.controllers.servo.RGBIndicatorLightController;
import org.firstinspires.ftc.teamcode.hardware.controllers.servo.ServoTimerController;
import org.firstinspires.ftc.teamcode.hardware.controllers.subsystems.Turret;
import org.firstinspires.ftc.teamcode.hardware.drivetrain.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.robots.base.RobotBase;
import org.firstinspires.ftc.teamcode.robots.base.opmodes.OpModeBase;
import org.firstinspires.ftc.teamcode.util.LinearInterpolator;
import org.firstinspires.ftc.teamcode.util.LogicUtil;
import org.firstinspires.ftc.teamcode.util.info.HardwareInfo;
import org.firstinspires.ftc.teamcode.util.math.Angle;
import org.firstinspires.ftc.teamcode.util.math.LeadComputingUtil;

import java.util.Optional;
import java.util.TreeMap;

@Configurable
public class JetFireRobot extends RobotBase {
    private Turret turret;
    private boolean isAutoAimOn = false;

    public enum FlywheelMode {
        AUTO,
        SET_VELOCITY,
        OFF
    }
    private FlywheelMode flywheelMode = FlywheelMode.OFF;
    private static double flywheelSetVelocity = 3000;

    public enum IntakeMode {
        INTAKE,
        REVERSE,
        OFF
    }
    IntakeMode intakeMode = IntakeMode.OFF;

    DcMotorEx intakeMotor;
    private static final double INTAKE_POWER = 0.6;

    public static boolean isAutoHoodAngleOn = false;
    public static double hoodSetAngleDeg = 22;

    public static Pose targetGoal;
    // TODO: AIM GOAL POINT?
    private static Pose resetPose; // HUMAN PLAYER ZONE
    private Angle headingTowardsGoal;
    private double distanceFromGoal;
    private Angle relativeTurntableHeading;
    private Angle absoluteTurntableHeading;

    // TODO: Better way? rather than 3 different interpolators?
    private LinearInterpolator flywheelVelocityByDistanceInterpolator;
    private LinearInterpolator hoodAngleByDistanceInterpolator;
    private LinearInterpolator timeOfFlightByDistanceInterpolator;

    ServoTimerController launchServoController;
    private static final double LAUNCH_SERVO_UP = 0.7;
    private static final double LAUNCH_SERVO_DOWN = 0.27;
    private static final long LAUNCH_SERVO_DELAY_MS = 300;
    // TODO: TEST ANGLE CLASS PANELS

    RGBIndicatorLightController indicatorLightController;

    public static PIDFCoefficients flywheelPIDFCoefficients = new PIDFCoefficients(0.0027, 0, 0, 0.0002);
    public static PIDFCoefficients turntablePIDFCoefficients = new PIDFCoefficients(0.015 / 2, 0, 0, 0);

    public static double flywheelVelocityMargin = 60;
    public static double turntableHeadingMarginDeg = 2;

    @Override
    public void init(HardwareMap hardwareMap, Pose startPose, OpModeBase.AllianceColor allianceColor) {
        super.init(hardwareMap, startPose, allianceColor);
        targetGoal = new Pose(-72, allianceColor == OpModeBase.AllianceColor.RED ? 72 : -72);
        // targetAim
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
                        //HardwareInfo.GOBILDA_5203_YELLOW_JACKET_MOTOR_6000_RPM.ENCODER_RESOLUTION_PPR
                        28,
                        1
                ),
                new AngleMotorController(
                        hardwareMap.get(DcMotorEx.class, "turntable"),
                        DcMotorSimple.Direction.FORWARD,
                        turntablePIDFCoefficients,
                        384.5,
                        64.0 / 16.0,
                        new Angle(130, false),
                        new Angle(-130, false),
                        turretStartHeading,
                        true
                ),
                new AngleServoController(
                        hardwareMap.get(Servo.class, "hood"),
                        new Angle(300, false),
                        (48.0 / 40.0) * (116.0 / 12.0),
                        new Angle(16, false),
                        new Angle(36, false),
                        new Angle(16, false)
                )
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
        telemetry.addData("Hood Angle targ", hoodSetAngleDeg);
    }

    @Override
    public void startConfiguration() {

    }

    @Override
    public void update(long deltaTime) {
        Pose displacedPose = targetGoal.minus(follower.getPose());
        headingTowardsGoal = new Angle(Math.atan2(displacedPose.getY(), displacedPose.getX()));
        distanceFromGoal = follower.getPose().distanceFrom(targetGoal);
        relativeTurntableHeading = turret.turntableController().getHeading();
        absoluteTurntableHeading = new Angle(relativeTurntableHeading.getAngle(Angle.AngleSystem.SIGNED_180_WRAPPED) + follower.getHeading());



        // LinearInterpolator timeOfFlightInterpolator = new LinearInterpolator();

//        // Lead Computing
        boolean accountForFutureHeading;
        boolean accountForGalileanRelativity;
        boolean calculateFuturePose;

        long hardwareDelayMs = 150;
        long launchDelay = deltaTime + hardwareDelayMs;

        Pose currentPose = follower.getPose();
        Vector velocity = follower.getVelocity();
        Vector acceleration = follower.getAcceleration();

        Pose futurePose = LeadComputingUtil.getFuturePose(
                currentPose,
                velocity,
                acceleration,
                (launchDelay / 1000.0)
        );

        double distanceToGoal = futurePose.distanceFrom(targetGoal); // FROM GOAL
        double timeOfFlightMs = timeOfFlightByDistanceInterpolator.interpolate(distanceToGoal);
        // TODO: ALl in seconds?

        // TODO: Try to evulate everything at a static instance
        // TODO: idk

        //Pose virtualGoal = targetGoal.minus(new Pose(velocity.getXComponent() * timeOfFlightMs, velocity.getYComponent() * timeOfFlightMs));
        Pose virtualGoal = LeadComputingUtil.getRelativeVirtualTarget(targetGoal, velocity, (timeOfFlightMs / 1000.0));

        double virtualDistance = futurePose.distanceFrom(virtualGoal);

        // TODO: Feedforward on turntable

        // TODO: Max turntable power, maybe min too




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


        JetFireStaticData.lastTurretHeading = turret.turntableController().getHeading();


        indicatorLightController.setColor(isReadyToShoot() ? RGBIndicatorLightController.Color.GREEN : RGBIndicatorLightController.Color.RED);

//        launchServoController.update();

        // Testing
        if (flywheelPIDFCoefficients != turret.flywheelController().getPidfController().getCoefficients()) turret.flywheelController().getPidfController().setCoefficients(flywheelPIDFCoefficients);
        if (turntablePIDFCoefficients != turret.turntableController().getPidfController().getCoefficients()) turret.turntableController().getPidfController().setCoefficients(turntablePIDFCoefficients);

        super.update(deltaTime);
    }

    public void shoot() {
        // wait for reset
        launchServoController.setPosition(LAUNCH_SERVO_UP, 200, LAUNCH_SERVO_DOWN, 200);
    }

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
        boolean isFlywheelReady = LogicUtil.isWithinRange(
                turret.flywheelController().getVelocity(),
                turret.flywheelController().getTargetVelocity(),
                flywheelVelocityMargin
        ) && !flywheelMode.equals(FlywheelMode.OFF);

        boolean isTurntableReady = LogicUtil.isWithinRange(
                turret.turntableController().getHeading().getAngle(Angle.AngleUnit.DEGREES, Angle.AngleSystem.SIGNED_180_WRAPPED),
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
        return Constants.createJetFireFollower(hardwareMap);
    }

    @Override
    public Limelight3A instantiateLimelight3A(HardwareMap hardwareMap) {
        return null;
    }
}
