package org.firstinspires.ftc.teamcode.robots.decode;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.robots.base.RobotBase;

public class DecodeRobot extends RobotBase {
    enum Position {
        RED_OBLISK,
        BLUE_OBALISK,
        RED
    }

    private Limelight3A limelight;
    public DcMotorEx dcMotor;
    public DcMotorEx dcMotor2;

    @Override
    public boolean initHardware(HardwareMap hardwareMap) {
        dcMotor = hardwareMap.get(DcMotorEx.class, "motor1");
        dcMotor2 = hardwareMap.get(DcMotorEx.class, "motor2");


//        limelight = hardwareMap.get(Limelight3A.class, "limelight");
//        limelight.pipelineSwitch(0);
//        limelight.start();

        //initStandardHardware(hardwareMap);
        return true;
    }
}
