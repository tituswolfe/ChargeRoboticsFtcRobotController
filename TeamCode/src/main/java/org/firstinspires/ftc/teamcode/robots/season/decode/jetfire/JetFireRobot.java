package org.firstinspires.ftc.teamcode.robots.season.decode.jetfire;

import static org.firstinspires.ftc.teamcode.hardware.drivetrain.pedroPathing.Constants.createFollower;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.control.PIDFController;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.hardware.controllers.motor.RPSController;
import org.firstinspires.ftc.teamcode.hardware.controllers.servo.RGBIndicatorLightController;
import org.firstinspires.ftc.teamcode.hardware.controllers.servo.ServoTimerController;
import org.firstinspires.ftc.teamcode.robots.base.RobotBase;
import org.firstinspires.ftc.teamcode.robots.base.StaticData;
import org.firstinspires.ftc.teamcode.robots.base.opmodes.OpModeBase;
import org.firstinspires.ftc.teamcode.util.LinearInterpolator;
import org.firstinspires.ftc.teamcode.util.LogicUtil;

import java.util.TreeMap;

@Configurable
public class JetFireRobot extends RobotBase {
    public static final int BLUE_GOAL_AP_ID = 20;
    public static final int RED_GOAL_AP_ID = 24;

    public static final int OBELISK_GPP_AP_ID = 21;
    public static final int OBELISK_PGP_AP_ID = 22;
    public static final int OBELISK_PPG_AP_ID = 23;

    // FLYWHEELS
    public static final double topFlywheelRatio = 0.5;
    public double flywheelSpeed = 0;
    private RPSController topFlywheel;
    private RPSController bottomFlywheel;
    private LinearInterpolator flywheelSpeedByDistanceInterpolator;
    public enum FlywheelSpeedMode {
        AUTO,
        MANUEL,
        OFF
    } // CLOSE, AND FAR?
    private FlywheelSpeedMode flywheelSpeedMode = FlywheelSpeedMode.OFF;
    private static final double CLOSE_SHOT_FLYWHEEL_SPEED = 30;

    // INTAKE
    private static final double INTAKE_SPEED = 35;
    public RPSController intake;
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
    private static final double PUSH_SERVO_OUT = 0.3;
    private static final double PUSH_SERVO_IN = 0.575;

    // LIGHTS
    private RGBIndicatorLightController rgbIndicatorLightController;

    private Pose targetGoal;
    private Pose targetGoalAim;

    private PIDFController headingPIDFController;
    @Override
    public void startConfiguration() {
        launchServoController.getServo().setPosition(LAUNCH_SERVO_DOWN);
        pushServoController.getServo().setPosition(PUSH_SERVO_IN);
    }

    @Override
    public void initHardware(HardwareMap hardwareMap) {

        headingPIDFController = new PIDFController(follower.constants.coefficientsHeadingPIDF);

        TreeMap<Double, Double> flywheelSpeedByDistance = new TreeMap<>();
        flywheelSpeedByDistance.put(63.0, 30.0);
        flywheelSpeedByDistance.put(82.0, 30.0);
        flywheelSpeedByDistance.put(100.0, 30.0);
        flywheelSpeedByDistance.put(135.0, 41.0);

        flywheelSpeedByDistanceInterpolator = new LinearInterpolator(flywheelSpeedByDistance);

        // Flywheels
        DcMotorEx flywheelMotor1 = hardwareMap.get(DcMotorEx.class, "top-flywheel");
        flywheelMotor1.setDirection(DcMotorSimple.Direction.FORWARD);
        flywheelMotor1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(900, 5, 0, 0));

        DcMotorEx flywheelMotor2 = hardwareMap.get(DcMotorEx.class, "bottom-flywheel");
        flywheelMotor2.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheelMotor2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(900, 5, 0, 0));

        topFlywheel = new RPSController(flywheelMotor1, 28);
        bottomFlywheel = new RPSController(flywheelMotor2, 28);

        // Intake
        DcMotorEx intakeMotor = hardwareMap.get(DcMotorEx.class, "intake");
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        intake = new RPSController(intakeMotor, 384.5);

        // Servos
        launchServoController = new ServoTimerController(hardwareMap.get(Servo.class, "launch"));
        pushServoController = new ServoTimerController(hardwareMap.get(Servo.class, "thing1")); // TODO: Rename push servo

        // Other
        rgbIndicatorLightController = new RGBIndicatorLightController(hardwareMap.get(Servo.class, "indicator"));

        // TODO: Check for pedro vs ftc coords
        targetGoal = new Pose(
                -72,
                (StaticData.allianceColor == OpModeBase.AllianceColor.RED ? 72 : -72)
        );

        targetGoalAim = new Pose(
                -72,
                (StaticData.allianceColor == OpModeBase.AllianceColor.RED ? 92 : -55)
        );
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

    // Robot functions
// TODO: Override auto flywheel speeds

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
        intake.setRPS(switch (intakeMode) {
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

        if (bottomFlywheel.getConfiguredRPS() != flywheelSpeed) {
            bottomFlywheel.setRPS(flywheelSpeed);
            topFlywheel.setRPS(flywheelSpeed * topFlywheelRatio);
        }

        launchServoController.update();
        pushServoController.update();

        rgbIndicatorLightController.setColor(areFlywheelsReady() ? RGBIndicatorLightController.Color.GREEN : RGBIndicatorLightController.Color.RED);

        super.updateHardwareStates();
    }

    public boolean areFlywheelsReady() {
        return LogicUtil.isWithinRange(topFlywheel.getCurrentRPS(), flywheelSpeed * topFlywheelRatio,1.5) && LogicUtil.isWithinRange(bottomFlywheel.getCurrentRPS(), flywheelSpeed, 1.5);
    }

    // getters

    public RPSController getTopFlywheel() {
        return topFlywheel;
    }

    public RPSController getBottomFlywheel() {
        return bottomFlywheel;
    }

    public Pose getTargetGoal() {
        return targetGoal;
    }

    public Pose getTargetGoalAim() {
        return targetGoalAim;
    }

    public PIDFController getHeadingPIDFController() {
        return headingPIDFController;
    }
}
