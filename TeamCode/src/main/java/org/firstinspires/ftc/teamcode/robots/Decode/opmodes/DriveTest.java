package org.firstinspires.ftc.teamcode.robots.Decode.opmodes;

import static org.firstinspires.ftc.teamcode.util.ThreadUtil.sleep;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Gamepad;

@Autonomous(name="Drive Train Test", group="Test")
public class DriveTest extends BaseOpMode {
    double targetPos;
    double power; double difference; double Kd;
    @Override
    public void loop() {
        super.loop();
//        getRobot().getOdometry().update();
//        telemetry.addData("X", getRobot().getOdometry().getXPos(DistanceUnit.INCH));
//        telemetry.addData("Y", getRobot().getOdometry().getYPos(DistanceUnit.INCH));
//        telemetry.addData("H", getRobot().getOdometry().getHeading(AngleUnit.DEGREES, Odometry.AngleSystem.UNSIGNED));
//
//        telemetry.addData("XVel", getRobot().getOdometry().getXVel(DistanceUnit.INCH));
//        telemetry.addData("YVel", getRobot().getOdometry().getYVel(DistanceUnit.INCH));
//
//        telemetry.addData("Power", power);
//        telemetry.addData("Diff", difference);
//
//        telemetry.update();

        //stop();
    }

    @Override
    public void processGamepad1Input(Gamepad gamepad) {

    }

    @Override
    public void processGamepad2Input(Gamepad gamepad) {

    }


    @Override
    public void start() {
        //getRobot().extendSlide();
        //getRobot().testManuver();

        //getRobot().getDriveTrain().driveTo(0, 48, 0, 0.4);
    }
}
