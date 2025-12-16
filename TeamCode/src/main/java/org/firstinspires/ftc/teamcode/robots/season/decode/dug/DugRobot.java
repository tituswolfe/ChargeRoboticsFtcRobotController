package org.firstinspires.ftc.teamcode.robots.season.decode.dug;

import static org.firstinspires.ftc.teamcode.hardware.drivetrain.pedroPathing.Constants.createFollower;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.control.PIDFController;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.hardware.controllers.motor.VelocityMotorController;
import org.firstinspires.ftc.teamcode.hardware.controllers.servo.RGBIndicatorLightController;
import org.firstinspires.ftc.teamcode.hardware.controllers.servo.ServoTimerController;
import org.firstinspires.ftc.teamcode.robots.base.RobotBase;
import org.firstinspires.ftc.teamcode.util.LinearInterpolator;
import org.firstinspires.ftc.teamcode.util.LogicUtil;
import org.firstinspires.ftc.teamcode.util.math.Angle;

import java.util.TreeMap;

@Configurable
public class DugRobot extends RobotBase {
    public static final int BLUE_GOAL_AP_ID = 20;
    public static final int RED_GOAL_AP_ID = 24;

    public static final int OBELISK_GPP_AP_ID = 21;
    public static final int OBELISK_PGP_AP_ID = 22;
    public static final int OBELISK_PPG_AP_ID = 23;

    // FLYWHEELS
    public double flywheelSpeed = 0;
    public double topFlywheelRatio = 0;
    private VelocityMotorController topFlywheel;
    private VelocityMotorController bottomFlywheel;
    private LinearInterpolator flywheelSpeedByDistanceInterpolator;
    private LinearInterpolator topFlywheelRatioByDistanceInterpolator;

    public enum FlywheelSpeedMode {
        AUTO,
        MANUEL,
        OFF
    } // CLOSE, AND FAR?
    private FlywheelSpeedMode flywheelSpeedMode = FlywheelSpeedMode.OFF;
    private static final double CLOSE_SHOT_FLYWHEEL_SPEED = 1800;

    // INTAKE
    private static final double INTAKE_SPEED = 435;
    public VelocityMotorController intake;
    public enum IntakeMode {
        OFF,
        INTAKE,
        REVERSE
    }
    private IntakeMode intakeMode = IntakeMode.OFF;

    // SERVOS
    ServoTimerController launchServoController;
    private static final double LAUNCH_SERVO_UP = 0.7;
    private static final double LAUNCH_SERVO_DOWN = 0.27;

    ServoTimerController pushServoController;
    private static final double PUSH_SERVO_OUT = 0.41;
    private static final double PUSH_SERVO_IN = 0.555;

    // LIGHTS
    private RGBIndicatorLightController rgbIndicatorLightController;

    private Pose targetGoal;
    // private Pose targetGoalAim;
    private PIDFController headingPIDFController;
    public double headingGoalOffsetDeg = -15;
    private double goalHeadingError;

    @Override
    public void startConfiguration() {
    }

    @Override
    public void initHardware(HardwareMap hardwareMap) {
        headingPIDFController = new PIDFController(follower.constants.coefficientsHeadingPIDF);

        TreeMap<Double, Double> flywheelSpeedByDistance = new TreeMap<>();
        flywheelSpeedByDistance.put(60.0, 1800.0);
        flywheelSpeedByDistance.put(70.0, 1800.0);
        flywheelSpeedByDistance.put(80.0, 1800.0);
        flywheelSpeedByDistance.put(90.0, 1800.0);
        flywheelSpeedByDistance.put(100.0, 1900.0);
        flywheelSpeedByDistance.put(110.0, 2000.0);
        flywheelSpeedByDistance.put(120.0, 2100.0);
        flywheelSpeedByDistance.put(150.0, 2200.0);

        TreeMap<Double, Double> topFlywheelRatioByDistance = new TreeMap<>();
        topFlywheelRatioByDistance.put(100.0, 0.5);

        flywheelSpeedByDistanceInterpolator = new LinearInterpolator(flywheelSpeedByDistance);
        topFlywheelRatioByDistanceInterpolator = new LinearInterpolator(topFlywheelRatioByDistance);

        // Flywheels

        topFlywheel = new VelocityMotorController(hardwareMap.get(DcMotorEx.class, "top-flywheel"), DcMotorSimple.Direction.FORWARD, new com.pedropathing.control.PIDFCoefficients(300, 15, 20, 0), 28, 1);
        bottomFlywheel = new VelocityMotorController(hardwareMap.get(DcMotorEx.class, "bottom-flywheel"), DcMotorSimple.Direction.REVERSE, new com.pedropathing.control.PIDFCoefficients(300, 15, 20, 0), 28, 1);

        // Intake
        DcMotorEx intakeMotor = hardwareMap.get(DcMotorEx.class, "intake");
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        intake = new VelocityMotorController(intakeMotor, DcMotorSimple.Direction.REVERSE, new com.pedropathing.control.PIDFCoefficients(300, 15, 20, 0),  384.5, 1); // DOESN'T NEED PID

        // Servos
        launchServoController = new ServoTimerController(hardwareMap.get(Servo.class, "launch"));
        pushServoController = new ServoTimerController(hardwareMap.get(Servo.class, "flicker")); // TODO: Rename push servo

        // Other
        rgbIndicatorLightController = new RGBIndicatorLightController(hardwareMap.get(Servo.class, "indicator"));



        launchServoController.setPosition(LAUNCH_SERVO_DOWN, 0, LAUNCH_SERVO_DOWN);
        pushServoController.setPosition(PUSH_SERVO_IN, 0, PUSH_SERVO_IN);
    }


    @Override
    protected FieldType instantiateFieldType() {
        return FieldType.SQUARE_INVERTED_ALLIANCE;
    }

    @Override
    public Follower instantiateFollower(HardwareMap hardwareMap) {
        return createFollower(hardwareMap);
    }

    @Override
    public Limelight3A instantiateLimelight3A(HardwareMap hardwareMap) {
        return null; // hardwareMap.get(Limelight3A.class, "limelight");
    }

    public FlywheelSpeedMode getFlywheelSpeedMode() {
        return flywheelSpeedMode;
    }

    public void setFlywheelSpeedMode(FlywheelSpeedMode flywheelSpeedMode) {
        this.flywheelSpeedMode = flywheelSpeedMode;
        if (flywheelSpeedMode == FlywheelSpeedMode.MANUEL) flywheelSpeed = CLOSE_SHOT_FLYWHEEL_SPEED;
    }

    public double getFlywheelSpeed() {
        return flywheelSpeed;
    }

    public void setIntakeMode(IntakeMode intakeMode) {
        this.intakeMode = intakeMode;
        intake.setTargetVelocity(switch (intakeMode) {
            case OFF -> 0.0;
            case INTAKE -> INTAKE_SPEED;
            case REVERSE -> -INTAKE_SPEED;
        });
    }

    public IntakeMode getIntakeMode() {
        return intakeMode;
    }

    public void launchArtifact() {
        launchServoController.setPosition(LAUNCH_SERVO_UP, 250, LAUNCH_SERVO_DOWN);
    }

    public void pushArtifact() {
        pushServoController.setPosition(PUSH_SERVO_OUT, 200, PUSH_SERVO_IN);
    }

    @Override
    public void updateHardwareStates() {
        // TODO: MORE INDICATOR LIGHTS WAHAHAHAHA!

        flywheelSpeed = switch (flywheelSpeedMode) {
            case AUTO -> flywheelSpeedByDistanceInterpolator.interpolate(follower.getPose().distanceFrom(targetGoal));
            case MANUEL -> flywheelSpeed;
            case OFF -> 0.0;
        };


        topFlywheelRatio = 0.5; //topFlywheelRatioByDistanceInterpolator.interpolate(follower.getPose().distanceFrom(targetGoal));

        if (bottomFlywheel.getVelocity() != flywheelSpeed) {
            bottomFlywheel.setTargetVelocity(flywheelSpeed);
            topFlywheel.setTargetVelocity(flywheelSpeed * topFlywheelRatio);
        }

        launchServoController.update();
        pushServoController.update();

        rgbIndicatorLightController.setColor(areFlywheelsReady() ? flywheelSpeedMode == FlywheelSpeedMode.AUTO ? RGBIndicatorLightController.Color.GREEN : RGBIndicatorLightController.Color.INDIGO : RGBIndicatorLightController.Color.RED);


        Pose displacedPose = targetGoal.minus(follower.getPose());
        double targetHeading = Math.atan2(displacedPose.getY(), displacedPose.getX()) + new Angle(headingGoalOffsetDeg, false).getAngle(Angle.AngleUnit.RADIANS, Angle.AngleSystem.SIGNED_180_WRAPPED); // Math.toRadians(-7);
        goalHeadingError = new Angle(targetHeading - follower.getHeading()).getAngle(Angle.AngleUnit.RADIANS, Angle.AngleSystem.SIGNED_180_WRAPPED);
        headingPIDFController.updateError(goalHeadingError);

        super.updateHardwareStates();
    }

    public boolean areFlywheelsReady() {
        if (flywheelSpeedMode == FlywheelSpeedMode.OFF) return false;
        return LogicUtil.isWithinRange(topFlywheel.getVelocity(), flywheelSpeed * topFlywheelRatio,60) && LogicUtil.isWithinRange(bottomFlywheel.getVelocity(), flywheelSpeed, 60);
    }

    // getters

    public VelocityMotorController getTopFlywheel() {
        return topFlywheel;
    }

    public VelocityMotorController getBottomFlywheel() {
        return bottomFlywheel;
    }

    public Pose getTargetGoal() {
        return targetGoal;
    }

    public double getGoalHeadingError() {
        return goalHeadingError;
    }

    public PIDFController getHeadingPIDFController() {
        return headingPIDFController;
    }

    public VelocityMotorController getIntake() {
        return intake;
    }
}
