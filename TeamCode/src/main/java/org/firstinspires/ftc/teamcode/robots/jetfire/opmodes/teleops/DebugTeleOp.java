package org.firstinspires.ftc.teamcode.robots.jetfire.opmodes.teleops;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@TeleOp(name = "JetFire Debug")
@Configurable
public class DebugTeleOp extends Teleop {
    public static double flywheelP = 100;
    public static double flywheelI = 5;
    public static double flywheelD = 0;
    public static double flywheelF = 0;

    @Override
    public void updateTelemetry(TelemetryManager telemetry) {
        Pose currentPose = robot.getFollower().getPose();
        telemetry.addData("angle toward goal", Math.toDegrees(Math.atan2(robot.redGoal.getY() - currentPose.getY(), robot.redGoal.getX() - currentPose.getX())));

        PIDFCoefficients currentPIDFCoefficients =  robot.getTopFlywheel().getDcMotorEx().getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addLine("- FLYWHEEL PIDs");
        telemetry.addData("flywheel P", currentPIDFCoefficients.p);
        telemetry.addData("flywheel I", currentPIDFCoefficients.i);
        telemetry.addData("flywheel D", currentPIDFCoefficients.d);
        telemetry.addData("flywheel F", currentPIDFCoefficients.f);
        telemetry.addLine("");

        super.updateTelemetry(telemetry);
    }

    @Override
    public void loop() {
        PIDFCoefficients currentPIDFCoefficients =  robot.getTopFlywheel().getDcMotorEx().getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        if (flywheelP != currentPIDFCoefficients.p || flywheelI != currentPIDFCoefficients.i || flywheelD != currentPIDFCoefficients.d || flywheelF != currentPIDFCoefficients.f) updateFlywheelPIDs();


        super.loop();
    }

    public void updateFlywheelPIDs() {
        robot.getTopFlywheel().getDcMotorEx().setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(flywheelP, flywheelI, flywheelD, flywheelF)
        );
        robot.getBottomFlywheel().getDcMotorEx().setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(flywheelP, flywheelI, flywheelD, flywheelF)
        );
    }
}
