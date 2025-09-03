package org.firstinspires.ftc.teamcode.robots.Decode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robots.ExampleRobot;

public class BaseLinearOpMode extends LinearOpMode {
    private ExampleRobot robot = new ExampleRobot("9808");

    @Override
    public void runOpMode() throws InterruptedException {
        while(opModeInInit()) {

        }
        waitForStart();
    }
}
