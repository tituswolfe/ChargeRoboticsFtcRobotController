package org.firstinspires.ftc.teamcode.robots.season.decode.jetfire;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.control.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.hardware.controllers.lights.Prism.Direction;
import org.firstinspires.ftc.teamcode.util.LinearInterpolator;
import org.firstinspires.ftc.teamcode.util.math.Angle;

import java.util.TreeMap;

@Configurable
public class JetFireConstants {
    // FLYWHEEL
    public static final String LEFT_FLYWHEEL_MOTOR_DEVICE_NAME = "left-flywheel";
    public static final DcMotorSimple.Direction LEFT_FLYWHEEL_MOTOR_DIRECTION = DcMotorSimple.Direction.REVERSE;
    public static final String RIGHT_FLYWHEEL_MOTOR_DEVICE_NAME = "right-flywheel";
    public static final DcMotorSimple.Direction RIGHT_FLYWHEEL_MOTOR_DIRECTION = DcMotorSimple.Direction.FORWARD;

    public static final String FLYWHEEL_NAME = "Flywheel";
    public static PIDFCoefficients FLYWHEEL_PIDF_COEFFICIENTS = new PIDFCoefficients(0.01, 0, 0, 0.00027);
    public static final double FLYWHEEL_GEAR_RATIO = 1;
    public static final double FLYWHEEL_MAX_POWER = 1;

    // TURNTABLE
    public static final String TURNTABLE_MOTOR_DEVICE_NAME = "turntable";
    public static final DcMotorSimple.Direction TURNTABLE_MOTOR_DIRECTION = DcMotorSimple.Direction.FORWARD;

    public static final String TURNTABLE_NAME = "Turntable";
    public static PIDFCoefficients TURNTABLE_PIDF_COEFFICIENTS = new PIDFCoefficients(0.025, 0, 0.001, -0.002); // i 0.000006
    public static final double TURNTABLE_TOTAL_GEAR_RATIO = 64.0 / 16.0;
    public static final double TURNTABLE_MAX_POWER = 0.75;
    public static final double TURNTABLE_MIN_HARD_STOP = Math.toRadians(-130);
    public static final double TURNTABLE_MAX_HARD_STOP = Math.toRadians(130);
    public static final boolean REVERSE_POWER = false;

    // HOOD
    public static final String HOOD_SERVO_DEVICE_NAME = "hood";
    public static final Servo.Direction HOOD_SERVO_DIRECTION = Servo.Direction.FORWARD;

    public static final String HOOD_NAME = "Hood";
    public static final Angle HOOD_TOTAL_ROTATION = new Angle(300, Angle.AngleUnit.DEGREES);
    public static final double HOOD_TOTAL_GEAR_RATIO = (48.0 / 40.0) * (225.0 / 22.0);
    public static final Angle MIN_HOOD_ANGLE = new Angle(26, Angle.AngleUnit.DEGREES);
    public static final Angle MAX_HOOD_ANGLE = new Angle(50, Angle.AngleUnit.DEGREES);

    // Regression (Deg) : Error (RPM)
    // 0.01 : 1
    // 1 : 100
    public static double HOOD_REGRESSION_RATIO = 0.035;

    // INTAKE
    public static final String INTAKE_MOTOR_DEVICE_NAME = "intake";
    public static final DcMotorSimple.Direction INTAKE_DIRECTION = DcMotorSimple.Direction.REVERSE;
    public static final double INTAKE_CURRENT_THRESHOLD_AMPS = 5.0;

    public static final double INTAKE_POWER = 1.0;
    public static final double REVERSE_INTAKE_POWER = -0.7;

    // TRANSFER
    public static double TRANSFER_SERVO_UP = 0.43; // 0.3
    public static double TRANSFER_SERVO_DOWN = 0.69;
    public static int TRANSFER_SERVO_TIME_MS = 150;

    // GATE
    public static double GATE_SERVO_OPEN = 0.6;
    public static double GATE_SERVO_CLOSED = 0.8;
    public static int GATE_SERVO_TIME_MS = 200;

    // COOLDOWN
    public static int LAUNCH_COOLDOWN_MS = 450;
    public static int INTAKE_COOLDOWN_MS = 0;

    // MARGINS & DELAYS
    public static double FLYWHEEL_VELOCITY_MARGIN_RPM = 200;
    public static double TURNTABLE_HEADING_MARGIN_DEG = 2;
    public static long LEAD_COMPUTING_TRANSFER_DELAY_MS = 80;

    // LIMELIGHT
    public static final int LIMELIGHT_LOCALIZATION_PIPELINE = 0;

    // LUTs
    private static final TreeMap<Double, Double> FLYWHEEL_VELOCITY_BY_DISTANCE_MAP = new TreeMap<>();
    static {
        // INCH, RPM
        FLYWHEEL_VELOCITY_BY_DISTANCE_MAP.put(30.0, 1950.0);
        FLYWHEEL_VELOCITY_BY_DISTANCE_MAP.put(36.0, 1950.0);
        FLYWHEEL_VELOCITY_BY_DISTANCE_MAP.put(42.0, 2100.0);
        FLYWHEEL_VELOCITY_BY_DISTANCE_MAP.put(54.0, 2150.0);
        FLYWHEEL_VELOCITY_BY_DISTANCE_MAP.put(66.0, 2200.0);
        FLYWHEEL_VELOCITY_BY_DISTANCE_MAP.put(78.0, 2350.0);
        FLYWHEEL_VELOCITY_BY_DISTANCE_MAP.put(90.0, 2500.0);
        FLYWHEEL_VELOCITY_BY_DISTANCE_MAP.put(96.0, 2550.0); // half
        FLYWHEEL_VELOCITY_BY_DISTANCE_MAP.put(102.0, 2600.0);
        FLYWHEEL_VELOCITY_BY_DISTANCE_MAP.put(108.0, 2800.0); // half
        FLYWHEEL_VELOCITY_BY_DISTANCE_MAP.put(114.0, 2900.0);
        FLYWHEEL_VELOCITY_BY_DISTANCE_MAP.put(120.0, 3000.0); // half
        FLYWHEEL_VELOCITY_BY_DISTANCE_MAP.put(126.0, 3000.0);
        FLYWHEEL_VELOCITY_BY_DISTANCE_MAP.put(132.0, 3000.0); // half
        FLYWHEEL_VELOCITY_BY_DISTANCE_MAP.put(140.0, 3000.0);
        FLYWHEEL_VELOCITY_BY_DISTANCE_MAP.put(150.0, 3300.0);
    }
    public static LinearInterpolator FLYWHEEL_VELOCITY_BY_DISTANCE = new LinearInterpolator(FLYWHEEL_VELOCITY_BY_DISTANCE_MAP);

    private static final TreeMap<Double, Double> HOOD_ANGLE_BY_DISTANCE_MAP = new TreeMap<>();
    static  {
        // INCH, DEGREES
        HOOD_ANGLE_BY_DISTANCE_MAP.put(30.0, 26.0);
        HOOD_ANGLE_BY_DISTANCE_MAP.put(36.0, 26.0);
        HOOD_ANGLE_BY_DISTANCE_MAP.put(42.0, 32.0);
        HOOD_ANGLE_BY_DISTANCE_MAP.put(54.0, 35.0);
        HOOD_ANGLE_BY_DISTANCE_MAP.put(66.0, 38.0);
        HOOD_ANGLE_BY_DISTANCE_MAP.put(78.0, 43.0);
        HOOD_ANGLE_BY_DISTANCE_MAP.put(90.0, 47.0);
        HOOD_ANGLE_BY_DISTANCE_MAP.put(96.0, 48.0); // half
        HOOD_ANGLE_BY_DISTANCE_MAP.put(102.0, 49.0);
        HOOD_ANGLE_BY_DISTANCE_MAP.put(108.0, 49.0); // half
        HOOD_ANGLE_BY_DISTANCE_MAP.put(114.0, 49.0);
        HOOD_ANGLE_BY_DISTANCE_MAP.put(120.0, 48.0); // half
        HOOD_ANGLE_BY_DISTANCE_MAP.put(126.0, 48.0);
        HOOD_ANGLE_BY_DISTANCE_MAP.put(132.0, 48.5); // half
        HOOD_ANGLE_BY_DISTANCE_MAP.put(140.0, 48.5);
        HOOD_ANGLE_BY_DISTANCE_MAP.put(150.0, 48.5);
    }
    public static LinearInterpolator HOOD_ANGLE_BY_DISTANCE = new LinearInterpolator(HOOD_ANGLE_BY_DISTANCE_MAP);

    private static final TreeMap<Double, Double> TIME_OF_FLIGHT_BY_DISTANCE_MAP = new TreeMap<>();
    static {
        // INCH, MILLS

        TIME_OF_FLIGHT_BY_DISTANCE_MAP.put(30.0, 220.0);
        TIME_OF_FLIGHT_BY_DISTANCE_MAP.put(36.0, 250.0);
        TIME_OF_FLIGHT_BY_DISTANCE_MAP.put(42.0, 330.0);
        TIME_OF_FLIGHT_BY_DISTANCE_MAP.put(54.0, 370.0);
        TIME_OF_FLIGHT_BY_DISTANCE_MAP.put(66.0, 400.0);
        TIME_OF_FLIGHT_BY_DISTANCE_MAP.put(78.0, 450.0);
        TIME_OF_FLIGHT_BY_DISTANCE_MAP.put(90.0, 500.0);
        TIME_OF_FLIGHT_BY_DISTANCE_MAP.put(96.0, 520.0); // half
        TIME_OF_FLIGHT_BY_DISTANCE_MAP.put(102.0, 600.0);
        TIME_OF_FLIGHT_BY_DISTANCE_MAP.put(108.0, 600.0); // half
        TIME_OF_FLIGHT_BY_DISTANCE_MAP.put(114.0, 600.0);
        TIME_OF_FLIGHT_BY_DISTANCE_MAP.put(120.0, 600.0); // half
        TIME_OF_FLIGHT_BY_DISTANCE_MAP.put(126.0, 600.0);
        TIME_OF_FLIGHT_BY_DISTANCE_MAP.put(132.0, 600.0); // half
        TIME_OF_FLIGHT_BY_DISTANCE_MAP.put(140.0, 600.0);
        TIME_OF_FLIGHT_BY_DISTANCE_MAP.put(150.0, 600.0);
    }
    public static LinearInterpolator TIME_OF_FLIGHT_BY_DISTANCE = new LinearInterpolator(TIME_OF_FLIGHT_BY_DISTANCE_MAP);

    // OTHER
    public static final double FAR_ZONE_X_THRESHOLD = 24;
}
