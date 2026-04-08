package org.firstinspires.ftc.teamcode.hardware.controllers.motor;

import androidx.core.math.MathUtils;

import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.hardware.controllers.HardwareController;
import org.firstinspires.ftc.teamcode.util.info.MotorInfo;
import org.firstinspires.ftc.teamcode.util.math.MathUtil;

public abstract class PIDFMotorController extends MotorController {
    protected PIDFController pidfController; // Pedro Pathing

    public PIDFMotorController(DcMotorEx device, String name, MotorInfo motorInfo, double totalGearRatio, double maxPower, PIDFCoefficients pidfCoefficients) {
        super(device, name, motorInfo, totalGearRatio, maxPower);

        pidfController = new PIDFController(pidfCoefficients);
    }


    public PIDFCoefficients getPIDFCoefficients() {
        return pidfController.getCoefficients();
    }

    public void setPIDFCoefficients(PIDFCoefficients pidfCoefficients) {
        pidfController.setCoefficients(pidfCoefficients);
    }
}
