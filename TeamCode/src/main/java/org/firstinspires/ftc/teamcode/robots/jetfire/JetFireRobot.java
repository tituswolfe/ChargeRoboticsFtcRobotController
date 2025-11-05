package org.firstinspires.ftc.teamcode.robots.jetfire;

import static org.firstinspires.ftc.teamcode.hardware.drivetrain.pedroPathing.Constants.createFollower;
import static org.firstinspires.ftc.teamcode.robots.base.opmodes.OpModeBase.activeSleep;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.hardware.controllers.motor.RPSController;
import org.firstinspires.ftc.teamcode.robots.base.RobotBase;

@Configurable
public class JetFireRobot extends RobotBase {
    public static final int redGoalAprilTagID = 22;
    public static final int blueGoalAprilTagID = 22;
    public static final int purpleGreenPorpleMotif = 22;


    // Hardware
    public static final double topFlywheelBaseSpeed = 15;
    public static final double bottomFlywheelBaseSpeed = 30;

    public double flywheelSpeedFactor = 0;

    private RPSController topFlywheel;
    private RPSController bottomFlywheel;

    public enum FlywheelDistancePreset {
        OFF,
        CLOSE,
        FAR
    }
    public FlywheelDistancePreset flywheelDistancePreset = FlywheelDistancePreset.OFF;

    public final double intakeSpeed = 35;
    public RPSController intake;

    public Servo lanchServo;
    public Servo thing1;

    public Servo indicator;

    // Poses
    public Pose blueAutoStart = new Pose(-60, -45, Math.toRadians(-124));
    public Pose blueCloseShoot = new Pose(-33, -18.8, Math.toRadians(-126.5));

    Pose redGoal = new Pose(-72, 72);


    public PathChain shoot1;

    @Override
    public void startConfiguration() {

    }

    @Override
    public void buildPaths(PathBuilder pathBuilder) {
        shoot1 = pathBuilder
                .addPath(new BezierLine(blueAutoStart, blueCloseShoot))
                .setLinearHeadingInterpolation(blueAutoStart.getHeading(), blueCloseShoot.getHeading())
                .build();

    }

    @Override
    public void initHardware(HardwareMap hardwareMap) {
        DcMotorEx flywheelMotor1 = hardwareMap.get(DcMotorEx.class, "top-flywheel");
        flywheelMotor1.setDirection(DcMotorSimple.Direction.FORWARD);
        flywheelMotor1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(50, 3, 100, 0));

        DcMotorEx flywheelMotor2 = hardwareMap.get(DcMotorEx.class, "bottom-flywheel");
        flywheelMotor2.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheelMotor2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(50, 3, 100, 0));

        topFlywheel = new RPSController(flywheelMotor1, 28);
        bottomFlywheel = new RPSController(flywheelMotor2, 28);

        DcMotorEx intakeMotor = hardwareMap.get(DcMotorEx.class, "intake");
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        intake = new RPSController(intakeMotor, 384.5);

        lanchServo = hardwareMap.get(Servo.class, "launch");
        thing1 = hardwareMap.get(Servo.class, "thing1");
        thing1.setPosition(0.625);

        indicator = hardwareMap.get(Servo.class, "indicator");
        indicatorRed();
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
    public void indicatorRed() {
        indicator.get
        indicator.setPosition(0.3);
    }
    public void indicatorGreen() {
        indicator.setPosition(0.5);
    }

    public void indicatorIndigo() {
        indicator.setPosition(0.666);
    }

    public void startFlywheels(double flywheelSpeedFactor) {
        this.flywheelSpeedFactor = flywheelSpeedFactor;
        topFlywheel.setRPS(topFlywheelBaseSpeed * flywheelSpeedFactor);
        bottomFlywheel.setRPS(bottomFlywheelBaseSpeed * flywheelSpeedFactor);
    }
    public void stopFlywheels() {
        topFlywheel.setRPS(0);
        bottomFlywheel.setRPS(0);
    }

    public double getFlywheelSpeedFactor() {
        return flywheelSpeedFactor;
    }

    public void toggleFlywheelPreset() {
        switch (flywheelDistancePreset) {
            case OFF:
                startFlywheels(1);
                flywheelDistancePreset = FlywheelDistancePreset.CLOSE;
                break;
            case CLOSE:
                startFlywheels(1.25);
                flywheelDistancePreset = FlywheelDistancePreset.FAR;
                break;
            case FAR:
                stopFlywheels();
                flywheelDistancePreset = FlywheelDistancePreset.OFF;
                break;
        }
    }

    public void startIntake() {
        intake.setRPS(intakeSpeed);
    }
    public void stopIntake() {
        intake.setRPS(0);
    }
    public void reverseIntake() {
        intake.setRPS(-intakeSpeed);
    }

    public void launchArtifact() {
        lanchServo.setPosition(0.7);
        activeSleep(400);
        lanchServo.setPosition(0.27);
    }

    public void flicker() {
        thing1.setPosition(0.3);
        activeSleep(250);
        thing1.setPosition(0.625);
    }

    // getters

    public RPSController getTopFlywheel() {
        return topFlywheel;
    }

    public RPSController getBottomFlywheel() {
        return bottomFlywheel;
    }
}
