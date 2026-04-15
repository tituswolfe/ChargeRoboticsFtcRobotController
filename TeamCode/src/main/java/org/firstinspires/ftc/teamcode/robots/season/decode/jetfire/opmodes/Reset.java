package org.firstinspires.ftc.teamcode.robots.season.decode.jetfire.opmodes;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robots.base.StaticData;
import org.firstinspires.ftc.teamcode.robots.season.decode.jetfire.JetfireStaticData;
import org.firstinspires.ftc.teamcode.util.math.Angle;

@TeleOp(name = "RESET")
public class Reset extends OpMode {
    @Override
    public void init() {
        JetfireStaticData.lastTurretHeading = 0;
        StaticData.lastPose = new Pose(72, 72, Math.toRadians(-90));
        telemetry.addLine("Turntable zeroed!");
        telemetry.addLine("Position reset!");
    }

    @Override
    public void start() {
        requestOpModeStop();
    }

    @Override
    public void loop() {

    }
}
