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
    PIDFController pidfController = new PIDFController(new PIDFCoefficients(0.001, 0, 0, 0));
    private Angle targetHeading = new Angle(0);

    public AngleMotorController(DcMotorEx dcMotorEx, double ticksPerRevolution, double totalGearRatio, Angle maxPositiveLimit, Angle minNegativeLimit) {
        super(dcMotorEx, ticksPerRevolution, totalGearRatio);
        this.maxPositiveLimit = maxPositiveLimit;
        this.minNegativeLimit = minNegativeLimit;

        // TODO: Throw error for limits

        this.dcMotorEx.setDirection(DcMotorSimple.Direction.FORWARD);
        this.dcMotorEx.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.dcMotorEx.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setTargetHeading(Angle angle) {
        targetHeading = angle;

//        if (targetHeading > maxPositiveLimit.getAngle(Angle.AngleUnit.DEGREES))
//        targetHeading = new Angle(Math.min(
//                Math.max(
//                        angle.getAngle(Angle.AngleUnit.RADIANS, Angle.AngleSystem.SIGNED),
//                        minNegativeLimit.getAngle(Angle.AngleUnit.RADIANS, Angle.AngleSystem.SIGNED)),
//                maxPositiveLimit.getAngle(Angle.AngleUnit.RADIANS, Angle.AngleSystem.SIGNED)
//        ));
    }

    public void update() {
        pidfController.updateError(
                new Angle(
                        targetHeading.getAngle(Angle.AngleUnit.DEGREES, Angle.AngleSystem.SIGNED) -  getAngle().getAngle(Angle.AngleUnit.DEGREES, Angle.AngleSystem.SIGNED),
                        false
                ).getAngle(Angle.AngleUnit.DEGREES, Angle.AngleSystem.SIGNED)
        );
        this.dcMotorEx.setPower(-pidfController.run());
    }

    public Angle getTargetHeading() {
        return new Angle(dcMotorEx.getTargetPosition() / ticksPerOutputRadian);
    }

    public Angle getAngle() {
        return new Angle(dcMotorEx.getCurrentPosition() / ticksPerOutputRadian);
    }
}
