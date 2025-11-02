package org.firstinspires.ftc.teamcode.robots.decode;

import static org.firstinspires.ftc.teamcode.hardware.drivetrain.pedroPathing.Constants.createFollower;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.hardware.controllers.motor.RPSController;
import org.firstinspires.ftc.teamcode.robots.base.RobotBase;
import org.firstinspires.ftc.teamcode.util.ThreadUtil;

public class DecodeRobot extends RobotBase {
    public static final int redGoalAprilTagID = 22;
    public static final int blueGoalAprilTagID = 22;
    public static final int purpleGreenPorpleMotif = 22;

    public RPSController flywheel1;
    public RPSController flywheel2;
    public RPSController intake;
    public Servo lanchServo;
    public Servo thing1;

    public Pose autoStart = new Pose(-60.7, -42.3, Math.toRadians(-126));
    public Pose closeShoot = new Pose(-41.9, -25.67, Math.toRadians(-131.1));

    public PathChain shoot1;

    @Override
    public void startConfiguration() {

    }

    @Override
    public void buildPaths(PathBuilder pathBuilder) {
        shoot1 = pathBuilder
                .addPath(new BezierLine(autoStart, closeShoot))
                .setLinearHeadingInterpolation(autoStart.getHeading(), closeShoot.getHeading())
                .build();

    }

    @Override
    public void initHardware(HardwareMap hardwareMap) {
        DcMotorEx flywheelMotor1 = hardwareMap.get(DcMotorEx.class, "top-flywheel");
        flywheelMotor1.setDirection(DcMotorSimple.Direction.FORWARD);

        DcMotorEx flywheelMotor2 = hardwareMap.get(DcMotorEx.class, "bottom-flywheel");
        flywheelMotor2.setDirection(DcMotorSimple.Direction.REVERSE);

        flywheel1 = new RPSController(flywheelMotor1, 28);
        flywheel2 = new RPSController(flywheelMotor2, 28);

        DcMotorEx intakeMotor = hardwareMap.get(DcMotorEx.class, "intake");
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        intake = new RPSController(intakeMotor, 384.5);

        lanchServo = hardwareMap.get(Servo.class, "launch");
        thing1 = hardwareMap.get(Servo.class, "thing1");
        thing1.setPosition(0.625);
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
    public void launchArtifact() {
        lanchServo.setPosition(0.7);
        ThreadUtil.sleep(500);
        lanchServo.setPosition(0.27);
    }

    public void runIntake() {
        intake.setRPS(25);
    }

    public void stopIntake() {
        intake.setRPS(0);
    }

    public void reverseIntake() {
        intake.setRPS(-25);
    }

    public void startFlywheels() {
        flywheel1.setRPS(20);
        flywheel2.setRPS(35);
    }


    public void stopFlywheels() {
        flywheel1.setRPS(0);
        flywheel2.setRPS(0);
    }

    public void flicker() {
        thing1.setPosition(0.3);
        ThreadUtil.sleep(250);
        thing1.setPosition(0.625);
    }


}
