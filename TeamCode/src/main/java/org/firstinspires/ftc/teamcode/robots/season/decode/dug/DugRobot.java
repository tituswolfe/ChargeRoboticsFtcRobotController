package org.firstinspires.ftc.teamcode.robots.season.decode.dug;

import static org.firstinspires.ftc.teamcode.hardware.drivetrain.pedroPathing.Constants.createDecodeFollower;

import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.hardware.controllers.motor.AngleMotorController;
import org.firstinspires.ftc.teamcode.hardware.controllers.motor.VelocityMotorController;
import org.firstinspires.ftc.teamcode.hardware.controllers.servo.AngleServoController;
import org.firstinspires.ftc.teamcode.hardware.controllers.servo.ServoController;
import org.firstinspires.ftc.teamcode.hardware.controllers.servo.ServoTimerController;
import org.firstinspires.ftc.teamcode.hardware.controllers.subsystems.Turret;
import org.firstinspires.ftc.teamcode.robots.base.RobotBase;
import org.firstinspires.ftc.teamcode.robots.base.StaticData;
import org.firstinspires.ftc.teamcode.robots.base.opmodes.OpModeBase;
import org.firstinspires.ftc.teamcode.robots.season.decode.jetfire.JetFireRobot;
import org.firstinspires.ftc.teamcode.util.LinearInterpolator;
import org.firstinspires.ftc.teamcode.util.math.Angle;

import java.util.TreeMap;

public class DugRobot extends RobotBase {
    public Turret turret;

    enum TurretMode {
        SET_ANGLE,
        AIM_AT_GOAL
    }

    public enum IntakeMode {
        INTAKE,
        REVERSE,
        OFF
    }
    IntakeMode intakeMode;

    private Pose targetGoal;
    private Angle headingTowardsGoal;
    private double distanceFromGoal;
    private Angle relativeTurntableHeading;
    private Angle absoluteTurntableHeading;

    private LinearInterpolator flywheelSpeedByDistanceInterpolator;
    private LinearInterpolator hoodAngleByDistanceInterpolator;

    ServoTimerController launchServoController;
    private static final double LAUNCH_SERVO_UP = 0.7;
    private static final double LAUNCH_SERVO_DOWN = 0.27;

    VelocityMotorController intakeController;
    private static final double INTAKE_SPEED = 435;

    @Override
    public void initHardware(HardwareMap hardwareMap) {
        turret = new Turret(
                new VelocityMotorController(hardwareMap.get(DcMotorEx.class, "flywheel"), 28, 1),
                new AngleMotorController(
                        hardwareMap.get(DcMotorEx.class, "turntable"),
                        384.5,
                        (double) 64 / 16,
                        new Angle(130, false),
                        new Angle(-130, false)
                ),
                new AngleServoController(
                        hardwareMap.get(Servo.class, "hood"),
                        new Angle(0),
                        new Angle(22, false),
                        new Angle(22 + 19, false),
                        new Angle(22)
                )
        );

//
//
//        DcMotorEx intakeMotor = hardwareMap.get(DcMotorEx.class, "intake");
//        //intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//        intakeController = new VelocityMotorController(intakeMotor, 384.5, 1);
//
//        TreeMap<Double, Double> flywheelSpeedByDistanceMap = new TreeMap<>();
//        flywheelSpeedByDistanceMap.put(60.0, 1800.0);
//        flywheelSpeedByDistanceInterpolator = new LinearInterpolator(flywheelSpeedByDistanceMap);
//
//        TreeMap<Double, Double> hoodAngleByDistanceMap = new TreeMap<>();
//        hoodAngleByDistanceMap.put(60.0, 18.5); // TODO: RADIANS
//        hoodAngleByDistanceInterpolator = new LinearInterpolator(hoodAngleByDistanceMap);
    }

    @Override
    public void hardwareTelemetry(TelemetryManager telemetry) {
        super.hardwareTelemetry(telemetry);
        telemetry.addLine("");
        telemetry.addLine("- TURRET -");
        telemetry.addData("Heading Towards Goal", headingTowardsGoal.getAngle(Angle.AngleUnit.DEGREES, Angle.AngleSystem.SIGNED));
        telemetry.addData("Turret Heading Error", headingTowardsGoal.getAngle(Angle.AngleUnit.DEGREES, Angle.AngleSystem.SIGNED) - absoluteTurntableHeading.getAngle(Angle.AngleUnit.DEGREES, Angle.AngleSystem.SIGNED));
        telemetry.addData("Distance From Goal", distanceFromGoal);
        telemetry.addData("Relative Turntable Heading", relativeTurntableHeading.getAngle(Angle.AngleUnit.DEGREES, Angle.AngleSystem.SIGNED));
        telemetry.addData("Absolute Turntable Heading", absoluteTurntableHeading.getAngle(Angle.AngleUnit.DEGREES, Angle.AngleSystem.SIGNED));

    }

    @Override
    public void setAllianceColor(OpModeBase.AllianceColor allianceColor) {
        targetGoal = new Pose(-72, allianceColor == OpModeBase.AllianceColor.RED ? 72 : -72);
        super.setAllianceColor(allianceColor);
    }

    @Override
    public void startConfiguration() {

    }

    @Override
    public void updateHardwareStates() {
        Pose displacedPose = targetGoal.minus(follower.getPose());
        headingTowardsGoal = new Angle(Math.atan2(displacedPose.getY(), displacedPose.getX()));
        distanceFromGoal = follower.getPose().distanceFrom(targetGoal);
        relativeTurntableHeading = turret.getTurntableController().getAngle();
        absoluteTurntableHeading = new Angle(turret.getTurntableController().getAngle().getAngle(Angle.AngleUnit.RADIANS, Angle.AngleSystem.SIGNED) + follower.getHeading());

        turret.getTurntableController().setTargetHeading(new Angle(headingTowardsGoal.getAngle(Angle.AngleSystem.SIGNED) - follower.getHeading()));
        turret.getTurntableController().update();

        //turret.getHoodServoController().setTargetAngle(new Angle(25, false));
        turret.getFlywheelController().setTargetVelocity(3000);

//        turret.getHoodServoController().setTargetAngle(new Angle(hoodAngleByDistanceInterpolator.interpolate(distanceFromGoal)));
//
//        launchServoController.update();

        super.updateHardwareStates();
    }

    public void shoot() {
        launchServoController.setPosition(LAUNCH_SERVO_UP, 200, LAUNCH_SERVO_DOWN);
    }

    public void setIntakeMode(IntakeMode intakeMode) {
        this.intakeMode = intakeMode;
        intakeController.setTargetVelocity(switch (intakeMode) {
            case OFF -> 0.0;
            case INTAKE -> INTAKE_SPEED;
            case REVERSE -> -INTAKE_SPEED;
        });
    }

    @Override
    protected FieldType instantiateFieldType() {
        return FieldType.SQUARE_INVERTED_ALLIANCE;
    }

    // TODO: STATIC???
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
