package org.firstinspires.ftc.teamcode.hardware.controllers.motor;

import com.pedropathing.control.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.util.math.Angle;

// without limit version
public class AngleMotorController extends MotorController {

    private final Angle maxPositiveLimit; // MAKE PRIVATE
    private final Angle minNegativeLimit;
    private final Angle startAngle;
    private final boolean reversePower;

    private Angle targetHeading = new Angle(0);


    public AngleMotorController(DcMotorEx dcMotorEx, DcMotorSimple.Direction direction, PIDFCoefficients pidfCoefficients, double ticksPerRevolution, double totalGearRatio, Angle maxPositiveLimit, Angle minNegativeLimit, Angle startAngle, boolean reversePower) {
        super(dcMotorEx, direction, pidfCoefficients, ticksPerRevolution, totalGearRatio);

        assert maxPositiveLimit.getAngle(Angle.AngleSystem.SIGNED_180_WRAPPED) > minNegativeLimit.getAngle(Angle.AngleSystem.SIGNED_180_WRAPPED);
        this.maxPositiveLimit = maxPositiveLimit;
        this.minNegativeLimit = minNegativeLimit;

        assert startAngle.getAngle(Angle.AngleSystem.SIGNED_180_WRAPPED) < maxPositiveLimit.getAngle(Angle.AngleSystem.SIGNED_180_WRAPPED);
        assert startAngle.getAngle(Angle.AngleSystem.SIGNED_180_WRAPPED) > minNegativeLimit.getAngle(Angle.AngleSystem.SIGNED_180_WRAPPED);
        this.startAngle = startAngle;

        this.reversePower = reversePower;

        this.dcMotorEx.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void update() {
        // TODO: Add other componets to PIDF Controller
        pidfController.updateError(getConstrainedTurn(targetHeading.getAngle(Angle.AngleUnit.DEGREES, Angle.AngleSystem.SIGNED_180_WRAPPED),  getAngle().getAngle(Angle.AngleUnit.DEGREES, Angle.AngleSystem.SIGNED_180_WRAPPED)));
        pidfController.updateFeedForwardInput(targetHeading.getAngle(Angle.AngleUnit.DEGREES, Angle.AngleSystem.SIGNED_180_WRAPPED));
        pidfController.updatePosition(getAngle().getAngle(Angle.AngleUnit.DEGREES, Angle.AngleSystem.SIGNED_180_WRAPPED));

        double power =  pidfController.run();
        this.dcMotorEx.setPower(reversePower ? -power : power);
    }

    public void setTargetHeading(Angle targetHeading) {
        this.targetHeading = new Angle(Math.min(
                maxPositiveLimit.getAngle(Angle.AngleSystem.SIGNED_180_WRAPPED),
                Math.max(
                        minNegativeLimit.getAngle(Angle.AngleSystem.SIGNED_180_WRAPPED),
                        targetHeading.getAngle(Angle.AngleSystem.SIGNED_180_WRAPPED)
                )
        ));
    }


    /**
     * Calculates the required turn (error) to reach the target,
     * guaranteeing the path does NOT cross the hardstop seam (130/-130 boundary).
     * * @param targetAngle The desired angle, normalized to the safe range.
     * @param currentAngle The current angle, normalized to the safe range.
     * @return The angular distance to turn, which avoids the hardstop.
     */
    public double getConstrainedTurn(double targetAngle, double currentAngle) {

        // NOTE: This assumes targetAngle and currentAngle are already within
        // the safe range [-130, 130]. If they are calculated outside this range,
        // you MUST clip them first to prevent errors.

        // Get the absolute shortest path error
        // For angles within [-130, 130], the shortest error is always correct.
        double shortestError = getAngleError(targetAngle, currentAngle);

        // --- Hardstop Constraint Logic ---

        // We only need to constrain the turn if the shortest path would take us
        // outside the safe range [-130, 130].
        double predictedPosition = currentAngle + shortestError;

        // Scenario 1: Shortest path goes beyond the MAX hardstop (130)
        // AND the shortest path turn is Clockwise (positive error).
        if (predictedPosition > maxPositiveLimit.getAngle(Angle.AngleUnit.DEGREES, Angle.AngleSystem.SIGNED_180_WRAPPED) && shortestError > 0) {

            // This means the shortest path is CLOCKWISE, but it would crash.
            // The safe route is the longer COUNTER-CLOCKWISE path.

            // To find the long CCW path, we subtract 360 degrees from the error.
            // Example: If shortestError is +20 (crashes at 130), force it to -340.
            return shortestError - 360;
        }

        // Scenario 2: Shortest path goes beyond the MIN hardstop (-130)
        // AND the shortest path turn is Counter-Clockwise (negative error).
        if (predictedPosition < minNegativeLimit.getAngle(Angle.AngleUnit.DEGREES, Angle.AngleSystem.SIGNED_180_WRAPPED)  && shortestError < 0) {

            // This means the shortest path is COUNTER-CLOCKWISE, but it would crash.
            // The safe route is the longer CLOCKWISE path.

            // To find the long CW path, we add 360 degrees to the error.
            // Example: If shortestError is -20 (crashes at -130), force it to +340.
            return shortestError + 360;
        }

        // If the shortest path does not crash into either hardstop, it is safe.
        return shortestError;
    }

// NOTE: The getAngleError() function remains the standard one:
// public double getAngleError(double targetAngle, double currentAngle) { ... }

    /**
     * Calculates the shortest angular distance (error) between two angles.
     * The result is always between -180 and 180 degrees.
     *
     * @param targetAngle The desired angle, normalized to [-180, 180].
     * @param currentAngle The current angle, normalized to [-180, 180].
     * @return The shortest turn required.
     */
    public double getAngleError(double targetAngle, double currentAngle) {
        // 1. Find the raw error
        double error = targetAngle - currentAngle;

        // 2. Normalize the error to the range [-180, 180]
        // Loop to bring the error back into the 360-degree window
        while (error > 180) {
            error -= 360;
        }
        while (error < -180) {
            error += 360;
        }

        return error;
    }

    public Angle getTargetHeading() {
        return new Angle(dcMotorEx.getTargetPosition() / ticksPerOutputRadian);
    }

    public Angle getAngle() {
        return new Angle(dcMotorEx.getCurrentPosition() / ticksPerOutputRadian);
    }
}
