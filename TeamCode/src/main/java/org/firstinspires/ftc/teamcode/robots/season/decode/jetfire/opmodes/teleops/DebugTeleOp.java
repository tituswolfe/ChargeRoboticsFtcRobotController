package org.firstinspires.ftc.teamcode.robots.season.decode.jetfire.opmodes.teleops;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@TeleOp(name = "JetFire Debug")
@Configurable
public class DebugTeleOp extends Teleop {
    public static double flywheelP = 0;
    public static double flywheelI = 0;
    public static double flywheelD = 0;
    public static double flywheelF = 0;

    @Override
    public void updateTelemetry(TelemetryManager telemetry) {
        Pose currentPose = robot.getFollower().getPose();
        telemetry.addData("angle toward goal", Math.toDegrees(Math.atan2(robot.getTargetGoal().getY() - currentPose.getY(), robot.getTargetGoal().getX() - currentPose.getX())));
        telemetry.addData("Distance from goal", robot.getFollower().getPose());

        PIDFCoefficients flywheelPIDFCoefficients =  robot.getBottomFlywheel().getDcMotorEx().getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addLine("- FLYWHEEL PIDs");
        telemetry.addData("flywheel P", flywheelPIDFCoefficients.p);
        telemetry.addData("flywheel I", flywheelPIDFCoefficients.i);
        telemetry.addData("flywheel D", flywheelPIDFCoefficients.d);
        telemetry.addData("flywheel F", flywheelPIDFCoefficients.f);
        telemetry.addLine("");

        super.updateTelemetry(telemetry);
    }

    @Override
    public void start() {
        PIDFCoefficients flywheelPIDFCoefficients = robot.getBottomFlywheel().getDcMotorEx().getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelP = flywheelPIDFCoefficients.p;
        flywheelI = flywheelPIDFCoefficients.i;
        flywheelD = flywheelPIDFCoefficients.d;
        flywheelF = flywheelPIDFCoefficients.f;

        super.start();
    }

    @Override
    public void loop() {
        updateFlywheelPIDs();

        super.loop();
    }

    public void updateFlywheelPIDs() {
        PIDFCoefficients newFlywheelPIDFCoefficients = new PIDFCoefficients(flywheelP, flywheelI, flywheelD, flywheelF);
        if (robot.getBottomFlywheel().getDcMotorEx().getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER).equals(newFlywheelPIDFCoefficients)) return;

        robot.getTopFlywheel().getDcMotorEx().setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER,
                newFlywheelPIDFCoefficients
        );
        robot.getBottomFlywheel().getDcMotorEx().setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER,
                newFlywheelPIDFCoefficients
        );
    }
}
