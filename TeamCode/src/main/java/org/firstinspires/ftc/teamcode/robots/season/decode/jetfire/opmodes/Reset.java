package org.firstinspires.ftc.teamcode.robots.season.decode.jetfire.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robots.season.decode.jetfire.JetfireStaticData;
import org.firstinspires.ftc.teamcode.util.math.Angle;

@TeleOp(name = "RESET Turntable")
public class Reset extends OpMode {

    @Override
    public void init() {
        JetfireStaticData.lastTurretHeading = new Angle(0);
        telemetry.addLine("TURNTABLE HAS BEEN ZEROED");
    }

    @Override
    public void start() {
        requestOpModeStop();
    }

    @Override
    public void loop() {

    }
}
