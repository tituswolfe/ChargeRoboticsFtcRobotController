package org.firstinspires.ftc.teamcode.robots.season.decode.jetfire;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.control.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.util.LinearInterpolator;
import org.firstinspires.ftc.teamcode.util.math.Angle;

import java.util.TreeMap;

@Configurable
public class JetFireConstants {
    // FLYWHEEL
    public static final String FLYWHEEL_NAME = "Flywheel";
    public static PIDFCoefficients FLYWHEEL_PIDF_COEFFICIENTS = new PIDFCoefficients(0.007, 0, 0, 0.0002);
    public static final double FLYWHEEL_GEAR_RATIO = 1;
    public static final double FLYWHEEL_MAX_POWER = 1;

    // TURNTABLE
    public static final String TURNTABLE_NAME = "Turntable";
    public static PIDFCoefficients TURNTABLE_PIDF_COEFFICIENTS = new PIDFCoefficients(0.051, 0, 0.0012, 0);
    public static final double TURNTABLE_TOTAL_GEAR_RATIO = 64.0 / 16.0;
    public static final double TURNTABLE_MAX_POWER = 0.75;
    public static final double TURNTABLE_HARD_STOP = 130;

    // HOOD
    public static final String HOOD_NAME = "Hood";
    public static final Angle HOOD_TOTAL_ROTATION = new Angle(300, false);
    public static final double HOOD_TOTAL_GEAR_RATIO = (48.0 / 40.0) * (116.0 / 12.0);
    public static final Angle MIN_HOOD_ANGLE = new Angle(22, false);
    public static final Angle MAX_HOOD_ANGLE = new Angle(42, false);

    // INTAKE
    public static final String INTAKE_NAME = "Intake";
    public static final PIDFCoefficients INTAKE_PIDF_COEFFICIENTS = null;
    public static final int INTAKE_TOTAL_GEAR_RATIO = 1;
    public static final int INTAKE_MAX_POWER = 1;

    public static final double INTAKE_POWER = 1;
    public static final double REVERSE_INTAKE_POWER = -0.6;

    // TRANSFER
    public static double TRANSFER_SERVO_UP = 0.4;
    public static double TRANSFER_SERVO_DOWN = 0.67;
    public static int TRANSFER_SERVO_TIME_MS = 200;

    // GATE
    public static double GATE_SERVO_OPEN = 0.6;
    public static double GATE_SERVO_CLOSED = 0.8;
    public static int GATE_SERVO_TIME_MS = 300;

    // COOLDOWN
    public static int LAUNCH_COOLDOWN_MS = 400;
    public static int INTAKE_COOLDOWN_MS = 150;

    // MARGINS & DELAYS
    public static double FLYWHEEL_VELOCITY_MARGIN_RPM = 60;
    public static double TURNTABLE_HEADING_MARGIN_DEG = 2;
    public static long LEAD_COMPUTING_TRANSFER_DELAY_MS = 80;

    // LUTs
    private static final TreeMap<Double, Double> flywheelSpeedByDistanceMap = new TreeMap<>();
    static {
        // INCH, RPM
        flywheelSpeedByDistanceMap.put(33.0, 2000.0);
        flywheelSpeedByDistanceMap.put(59.0, 2200.0);
        flywheelSpeedByDistanceMap.put(74.0, 2400.0);
        flywheelSpeedByDistanceMap.put(98.0, 2500.0);
        flywheelSpeedByDistanceMap.put(113.0, 2600.0);
        flywheelSpeedByDistanceMap.put(123.0, 3000.0);
    }
    public static final LinearInterpolator FLYWHEEL_VELOCITY_BY_DISTANCE = new LinearInterpolator(flywheelSpeedByDistanceMap);

    private static final TreeMap<Double, Double> hoodAngleByDistanceMap = new TreeMap<>();
    static  {
        // INCH, DEGREES
        hoodAngleByDistanceMap.put(33.0, 24.0);
        hoodAngleByDistanceMap.put(59.0, 31.0);
        hoodAngleByDistanceMap.put(74.0, 38.0);
        hoodAngleByDistanceMap.put(98.0, 41.0);
        hoodAngleByDistanceMap.put(113.0, 41.0);
        hoodAngleByDistanceMap.put(123.0, 41.0);
    }
    public static final LinearInterpolator HOOD_ANGLE_BY_DISTANCE = new LinearInterpolator(hoodAngleByDistanceMap);

    private static final TreeMap<Double, Double> timeOfFlightByDistanceMap = new TreeMap<>();
    static {
        // INCH, MILLS
        timeOfFlightByDistanceMap.put(33.0, 500.0);
        timeOfFlightByDistanceMap.put(59.0, 640.0);
        timeOfFlightByDistanceMap.put(74.0, 730.0);
        timeOfFlightByDistanceMap.put(98.0, 750.0);
        timeOfFlightByDistanceMap.put(113.0, 760.0);
        timeOfFlightByDistanceMap.put(123.0, 770.0);
    }
    public static final LinearInterpolator TIME_OF_FLIGHT_BY_DISTANCE = new LinearInterpolator(timeOfFlightByDistanceMap);

    // OTHER
    public static final double FAR_ZONE_X_THRESHOLD = 24;
}
