package org.firstinspires.ftc.teamcode.robots.base;


import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.util.FtcDashboardUtil;
import org.firstinspires.ftc.teamcode.util.ThreadUtil;

public abstract class BaseOpMode<Robot extends RobotBase, GamepadController extends GamepadControllerBase> extends OpMode {
    protected Robot robot;
    protected GamepadController gamepadController;

    private MultipleTelemetry multipleTelemetry = FtcDashboardUtil.getMultipleTelemetry(telemetry);

    private boolean isOpModeSetup = false;

    public void setupOpMode(Robot robot) {
        this.robot = robot;
        isOpModeSetup = true;
    }


    /**
     * Override this method, and call setupOpMode(), before calling super().
     */
    @Override
    public void init() {
        if(!isOpModeSetup) {
            multipleTelemetry.addLine("OpMode was not setup. Please call setupOpMode() in init");
            multipleTelemetry.update();
            stop();
        }
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
        multipleTelemetry.addLine("- Charger Robotics: " + this.robot.getName() + " -");
        multipleTelemetry.update();
    }

    @Override
    public void start() {

    }

    @Override
    public void stop() {
        ThreadUtil.shutdown();
    }

    public Robot getRobot() {
        return robot;
    }
}
