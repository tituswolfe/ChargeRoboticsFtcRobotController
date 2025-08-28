package org.firstinspires.ftc.teamcode.robots.Decode.opmodes;


import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.robot.Robot;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robots.ExampleRobot;
import org.firstinspires.ftc.teamcode.util.FtcDashboardUtil;
import org.firstinspires.ftc.teamcode.util.ThreadUtil;

public abstract class BaseOpMode extends OpMode {
    private ExampleRobot robot = new ExampleRobot("9808");
    private MultipleTelemetry multipleTelemetry = FtcDashboardUtil.getMultipleTelemetry(telemetry);

    private boolean allowGamepad2DriverControls;
    long gamepad1UnlockTime = 0; // System time (in mills) when controls can unlock
    long gamepad2UnlockTime = 0;



    @Override
    public void init() {
//        runAsync(() -> {
//            while(!robot.isRobotReady()) {
//                telemetry.addLine("Please wait .");
//                telemetry.addLine(" _____ _                               \n" +
//                        "/  __ \\ |                              \n" +
//                        "| /  \\/ |__   __ _ _ __ __ _  ___ _ __ \n" +
//                        "| |   | '_ \\ / _` | '__/ _` |/ _ \\ '__|\n" +
//                        "| \\__/\\ | | | (_| | | | (_| |  __/ |   \n" +
//                        " \\____/_| |_|\\__,_|_|  \\__, |\\___|_|   \n" +
//                        "                        __/ |          \n" +
//                        "                       |___/    "
//                );
//
//                telemetry.update();
//                sleep(500);
//
//
//                telemetry.addLine("Please wait . .");
//                telemetry.addLine("______      _           _   _          \n" +
//                        "| ___ \\    | |         | | (_)         \n" +
//                        "| |_/ /___ | |__   ___ | |_ _  ___ ___ \n" +
//                        "|    // _ \\| '_ \\ / _ \\| __| |/ __/ __|\n" +
//                        "| |\\ \\ (_) | |_) | (_) | |_| | (__\\__ \\\n" +
//                        "\\_| \\_\\___/|_.__/ \\___/ \\__|_|\\___|___/\n" +
//                        "                                   "
//                );
//                telemetry.update();
//
//                sleep(500);
//                telemetry.addLine("Please wait . . .");
//                telemetry.addLine(" _____ _                               \n" +
//                        "/  __ \\ |                              \n" +
//                        "| /  \\/ |__   __ _ _ __ __ _  ___ _ __ \n" +
//                        "| |   | '_ \\ / _` | '__/ _` |/ _ \\ '__|\n" +
//                        "| \\__/\\ | | | (_| | | | (_| |  __/ |   \n" +
//                        " \\____/_| |_|\\__,_|_|  \\__, |\\___|_|   \n" +
//                        "                        __/ |          \n" +
//                        "                       |___/    "
//                );
//                telemetry.update();
//
//                sleep(500);
//            }
//            telemetry.addLine("Ready!");
//            telemetry.update();
//        });
        multipleTelemetry.addLine("Charger Robotics");
        multipleTelemetry.addLine("Please wait . . .");
        multipleTelemetry.update();
        ThreadUtil.init();
        robot.init(hardwareMap);
        multipleTelemetry.addLine("READY");
    }

    @Override
    public void loop() {

        // process gamepad controls
        processDrivingInput(gamepad1);
        if (allowGamepad2DriverControls) {
            processDrivingInput(gamepad2);
        }

        if(gamepad1UnlockTime < System.currentTimeMillis()) {
            processGamepad1Input(gamepad1);
        }
        if (gamepad1UnlockTime < System.currentTimeMillis()) {
            processGamepad2Input(gamepad2);
        }

        multipleTelemetry.addLine("- Charger Robotics: " + this.getRobot().getName() + " -");
        multipleTelemetry.update();
    }

    @Override
    public void start() {

    }

    @Override
    public void stop() {
        getRobot().getDriveTrain().disable();
        ThreadUtil.shutdown();
    }

    public ExampleRobot getRobot() {
        return robot;
    }

    // Gamepad
    public abstract void processGamepad1Input(Gamepad Gamepad);
    public abstract void processGamepad2Input(Gamepad gamepad);

    double driveSpeedMultiplier = 0.75;
    double turnSpeedMultiplier = 0.6;
    double minStickSensitivity = 0.05;

    boolean isFastMode = false;

    public void processDrivingInput(Gamepad gamepad) {
        if (gamepad.rightStickButtonWasPressed()) {
            isFastMode = !isFastMode;
            if (isFastMode) {
                driveSpeedMultiplier = 0.75;
                turnSpeedMultiplier = 0.6;
            } else {
                driveSpeedMultiplier = 1;
                turnSpeedMultiplier = 0.75;
            }
        }

        double x_speed = gamepad.left_stick_x > minStickSensitivity ? gamepad.left_stick_x * driveSpeedMultiplier : 0;
        double y_speed = gamepad.left_stick_y > minStickSensitivity ? gamepad.left_stick_y * driveSpeedMultiplier : 0;
        double r_speed = gamepad.right_stick_x > minStickSensitivity ? gamepad.right_stick_x * turnSpeedMultiplier : 0;

        double cosHeading = getRobot().getDriveTrain().getCosHeading();
        double sinHeading = getRobot().getDriveTrain().getSinHeading();

        robot.getDriveTrain().setMotorPowersThroughKinematicTransformation(
                robot.getDriveTrain().calculateRobotStrafeFromFieldVelocity(x_speed, cosHeading, y_speed, sinHeading),
                robot.getDriveTrain().calculateRobotForwardFromFieldVelocity(x_speed, cosHeading, y_speed, sinHeading),
                r_speed
        );
    }

    public void lockOutGamepad1() {
        lockOutGamepad1(200);
    }

    public void lockOutGamepad1(int durationMills) {
        gamepad1UnlockTime = System.currentTimeMillis() + durationMills;
    }

    public void lockOutGamepad2() {
        lockOutGamepad2(200);
    }

    public void lockOutGamepad2(int durationMills) {
        gamepad2UnlockTime = System.currentTimeMillis() + durationMills;
    }
}
