package org.firstinspires.ftc.teamcode.robots.season.decode.jetfire;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.control.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.util.math.Angle;

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
    public final static String HOOD_NAME = "Hood";
    public final static Angle HOOD_TOTAL_ROTATION = new Angle(300, false);
    public final static double HOOD_TOTAL_GEAR_RATIO = (48.0 / 40.0) * (116.0 / 12.0);
    public final static Angle MIN_HOOD_ANGLE = new Angle(22, false);
    public final static Angle MAX_HOOD_ANGLE = new Angle(42, false);

    // INTAKE
    public final static String INTAKE_NAME = "Intake";
    public static PIDFCoefficients INTAKE_PIDF_COEFFICIENTS = null;
    public static final int INTAKE_TOTAL_GEAR_RATIO = 1;
    public static final int INTAKE_MAX_POWER = 1;

    public final static double INTAKE_POWER = 1;
    public final static double REVERSE_INTAKE_POWER = -0.6;

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

    public static void populateData() {

    }
}
