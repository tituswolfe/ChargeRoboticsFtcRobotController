package org.firstinspires.ftc.teamcode.robots.decode;

import static org.firstinspires.ftc.teamcode.hardware.drivetrain.pedroPathing.Constants.createFollower;

import com.bylazar.graph.GraphManager;
import com.bylazar.graph.PanelsGraph;
import com.bylazar.panels.Panels;
import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.hardware.controllers.motor.RPSController;
import org.firstinspires.ftc.teamcode.hardware.controllers.servo.ServoPositionController;
import org.firstinspires.ftc.teamcode.robots.base.RobotBase;

import java.util.HashMap;
import java.util.Map;

public class DecodeRobot extends RobotBase {
    public static final int redGoalAprilTagID = 22;
    public static final int blueGoalAprilTagID = 22;
    public static final int purpleGreenPorpleMotif = 22;

    public RPSController flywheel1;
    public RPSController flywheel2;
    public RPSController intake;
    public Servo servo;

    public GraphManager graphManager = PanelsGraph.INSTANCE.getManager();

    enum LaunchServoPositions {
        LAUNCH,
        REST
    }
    public ServoPositionController<LaunchServoPositions> launchServo;

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

//        HashMap<LaunchServoPositions, Double> launchServoPositionsHashMap = new HashMap<>();
//        launchServoPositionsHashMap.put(LaunchServoPositions.LAUNCH, 0.6);
//        launchServoPositionsHashMap.put(LaunchServoPositions.REST, 0.27);
//        launchServo = new ServoPositionController<>(hardwareMap.get(Servo.class, "launch"), launchServoPositionsHashMap);
//
//        launchServo.setPosition(LaunchServoPositions.REST);
        servo = hardwareMap.get(Servo.class, "launch");
    }

    @Override
    public Follower instantiateFollower(HardwareMap hardwareMap) {
        return createFollower(hardwareMap);
    }

    @Override
    public Limelight3A instantiateLimelight3A(HardwareMap hardwareMap) {
        return null; // hardwareMap.get(Limelight3A.class, "limelight");
    }
}
