package org.firstinspires.ftc.teamcode.robots.decode;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robots.base.BaseOpMode;


@TeleOp(name = "DecodeTeleop Ahhhh")
public class DecodeTeleop extends BaseOpMode<DecodeRobot, NealsonGamepadController> {

    @Override
    public void init() {
        DecodeRobot decodeRobot9808 = new DecodeRobot();
        setupOpMode(decodeRobot9808, new NealsonGamepadController(decodeRobot9808, gamepad1, gamepad2));
        robot = decodeRobot9808;
        super.init();
    }

    Pose startPose = new Pose(0, 0, 0);
    Pose secondPose = new Pose(36, 12, 0);
    double power = -28;
    double power1 = 28;
    @Override
    public void start() {

//        robot.getFollower().activateDrive();
//        robot.getFollower().activateAllPIDFs();
//        PathChain path = robot.getFollower().pathBuilder()
//                .addPath(new BezierLine(startPose, secondPose))
//                .setLinearHeadingInterpolation(startPose.getHeading(), secondPose.getHeading())
//                .setTangentHeadingInterpolation()
//                .build();
//
//
//        robot.getFollower().followPath(path);
//        robot.getFollower().startTeleopDrive();
    }

    @Override
    public void loop() {
        if(gamepad1.a) {
            power += -14;
            try {
                Thread.sleep(200);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
        }

        if(gamepad1.b) {
            power += 14;
            try {
                Thread.sleep(200);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
        }


        if(gamepad1.dpad_up) {
            power1 += 14;
            try {
                Thread.sleep(200);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
        }

        if(gamepad1.dpad_down) {
            power1 += -14;
            try {
                Thread.sleep(200);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
        }

        robot.dcMotor.setVelocity(power);
        robot.dcMotor2.setVelocity(power1);

        telemetry.addData("bottom flywheel set ticks (ticks/sec)", power / 28);
        telemetry.addData("top flywheel set ticks", power1 / 28);

        telemetry.addData("bottom flywheel ticks", robot.dcMotor.getVelocity() / 28);
        telemetry.addData("top flywheel ticks", robot.dcMotor2.getVelocity() / 28);


        telemetry.addData("bottom flywheel pow", robot.dcMotor.getPower());
        telemetry.addData("top flywheel pow", robot.dcMotor2.getPower());

        telemetry.update();
        //robot.getFollower().update();
        //super.loop();
    }
}
