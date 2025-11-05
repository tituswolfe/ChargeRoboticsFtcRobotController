package org.firstinspires.ftc.teamcode.robots.jetfire.opmodes.teleops;

import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "JetFire Debug")
public class DebugTeleOp extends Teleop{
    @Override
    public void updateTelemetry(TelemetryManager telemetry) {
        telemetry.addLine("- FLYWHEEL PIDs");
        telemetry.addData("flywheel P", robot.getTopFlywheel().getDcMotorEx().getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER).p);
        telemetry.addData("flywheel I", robot.getTopFlywheel().getDcMotorEx().getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER).i);
        telemetry.addData("flywheel D", robot.getTopFlywheel().getDcMotorEx().getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER).d);
        telemetry.addData("flywheel F", robot.getTopFlywheel().getDcMotorEx().getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER).f);
        telemetry.addLine("");

        super.updateTelemetry(telemetry);
    }
}
