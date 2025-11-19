package org.firstinspires.ftc.teamcode.hardware.controllers.motor;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.util.math.Angle;

// without limit version
public class AngleMotorController extends MotorController {
    public final Angle maxPositiveLimit;
    public final Angle minNegativeLimit;

    public AngleMotorController(DcMotorEx dcMotorEx, double ticksPerRevolution, double totalGearRatio, Angle maxPositiveLimit, Angle minNegativeLimit) {
        super(dcMotorEx, ticksPerRevolution, totalGearRatio);
        this.maxPositiveLimit = maxPositiveLimit;
        this.minNegativeLimit = minNegativeLimit;

        // TODO: Throw error for limits

        this.dcMotorEx.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.dcMotorEx.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.dcMotorEx.setPower(1.0);
    }

    public void setTargetAngle(Angle angle) {
        double targetAngle = Math.min(
                Math.max(angle.getAngle(Angle.AngleUnit.RADIANS, Angle.AngleSystem.SIGNED), minNegativeLimit.angleInRadians),
                maxPositiveLimit.angleInRadians
        );

        dcMotorEx.setTargetPosition((int) (targetAngle * ticksPerOutputRadian));
    }

    public Angle getTargetAngle() {
        return new Angle(dcMotorEx.getTargetPosition() / ticksPerOutputRadian);
    }

    public Angle getAngle() {
        return new Angle(dcMotorEx.getCurrentPosition() / ticksPerOutputRadian);
    }
}
