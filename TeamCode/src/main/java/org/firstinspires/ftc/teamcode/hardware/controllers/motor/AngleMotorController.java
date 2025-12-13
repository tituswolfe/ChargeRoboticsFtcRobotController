package org.firstinspires.ftc.teamcode.hardware.controllers.motor;

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.util.math.Angle;

// without limit version
public class AngleMotorController extends MotorController {
    public final Angle maxPositiveLimit;
    public final Angle minNegativeLimit;
    PIDFController pidfController = new PIDFController(new PIDFCoefficients(0.015, 0, 0, 0));
    private Angle targetHeading = new Angle(0);
    private Angle error = new Angle(0);

    public AngleMotorController(DcMotorEx dcMotorEx, double ticksPerRevolution, double totalGearRatio, Angle maxPositiveLimit, Angle minNegativeLimit) {
        super(dcMotorEx, ticksPerRevolution, totalGearRatio);
        this.maxPositiveLimit = maxPositiveLimit;
        this.minNegativeLimit = minNegativeLimit;

        assert maxPositiveLimit.getAngle(Angle.AngleSystem.SIGNED) > minNegativeLimit.getAngle(Angle.AngleSystem.SIGNED);

        this.dcMotorEx.setDirection(DcMotorSimple.Direction.FORWARD);
        this.dcMotorEx.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.dcMotorEx.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setTargetHeading(Angle targetHeading) {

        // -130 - 130 =-260 ( go backwards )
        // 130 - -130 = 260

        // -130 - 0 = -130
        //error = new Angle(targetHeading.getAngle(Angle.AngleUnit.DEGREES, Angle.AngleSystem.SIGNED) -  getAngle().getAngle(Angle.AngleUnit.DEGREES, Angle.AngleSystem.SIGNED));
        error = new Angle(getConstrainedTurn(targetHeading.getAngle(Angle.AngleUnit.DEGREES, Angle.AngleSystem.SIGNED),  getAngle().getAngle(Angle.AngleUnit.DEGREES, Angle.AngleSystem.SIGNED)));

        if (targetHeading.getAngle(Angle.AngleSystem.SIGNED) >= maxPositiveLimit.getAngle(Angle.AngleSystem.SIGNED)) {
            this.targetHeading = maxPositiveLimit;
        } else if (targetHeading.getAngle(Angle.AngleSystem.SIGNED) < minNegativeLimit.getAngle(Angle.AngleSystem.SIGNED)) {
            this.targetHeading = minNegativeLimit;
        } else {
            this.targetHeading = targetHeading; // Use the multi-turn-aware target
        }
//
//        if (Math.abs(error) > Math.toRadians(180)) {
//            targetHeading = new Angle(0);
//        }
    }

    public void update() {
        pidfController.updateError(getConstrainedTurn(targetHeading.getAngle(Angle.AngleUnit.DEGREES, Angle.AngleSystem.SIGNED),  getAngle().getAngle(Angle.AngleUnit.DEGREES, Angle.AngleSystem.SIGNED)));

        double motorPower = -pidfController.run();


        this.dcMotorEx.setPower(motorPower);
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
        if (predictedPosition > 130 && shortestError > 0) {

            // This means the shortest path is CLOCKWISE, but it would crash.
            // The safe route is the longer COUNTER-CLOCKWISE path.

            // To find the long CCW path, we subtract 360 degrees from the error.
            // Example: If shortestError is +20 (crashes at 130), force it to -340.
            return shortestError - 360;
        }

        // Scenario 2: Shortest path goes beyond the MIN hardstop (-130)
        // AND the shortest path turn is Counter-Clockwise (negative error).
        if (predictedPosition < -130 && shortestError < 0) {

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
