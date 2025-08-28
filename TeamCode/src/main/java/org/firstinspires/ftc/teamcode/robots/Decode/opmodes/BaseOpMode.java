package org.firstinspires.ftc.teamcode.robots.Decode.opmodes;


import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robots.ExampleRobot;
import org.firstinspires.ftc.teamcode.util.FtcDashboardUtil;
import org.firstinspires.ftc.teamcode.util.ThreadUtil;

public class BaseOpMode extends OpMode {
    private ExampleRobot robot = new ExampleRobot("9808");
    private MultipleTelemetry multipleTelemetry = FtcDashboardUtil.getMultipleTelemetry(telemetry);

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

    }

    @Override
    public void start() {
    }

    @Override
    public void stop() {
        getRobot().getDriveTrain().disable();
        ThreadUtil.shutdown();

//        requestOpModeStop();
    }

    public ExampleRobot getRobot() {
        return robot;
    }
}
