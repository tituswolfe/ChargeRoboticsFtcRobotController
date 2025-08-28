package org.firstinspires.ftc.teamcode.hardware.extensions;

import static org.firstinspires.ftc.teamcode.util.ThreadUtil.sleep;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class LinearSlide {
    private final DcMotorEx dcMotorEx;
    private final double ticksPerInch;
    private int tickOffset = 0;

    public LinearSlide(DcMotorEx dcMotorEx, DcMotorSimple.Direction direction, double ticksPerInch) {
        this.dcMotorEx = dcMotorEx;
        this.ticksPerInch = ticksPerInch;

        this.dcMotorEx.setDirection(direction);
    }

    public double getPosition() {
        return dcMotorEx.getCurrentPosition() / ticksPerInch;
    }

    public void setPosition(double inch) {
        dcMotorEx.setTargetPosition((int) (ticksPerInch * inch) + tickOffset);
    }

    public double getPower() {
        return dcMotorEx.getPower();
    }

    public void setPower(double power) {
        dcMotorEx.setPower(power);
    }

    public void reset(int stallTimeMills, int tickOffset) {
        // Reset physical position
        dcMotorEx.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        dcMotorEx.setPower(-1);
        sleep(stallTimeMills);


        // Reset encoder
        dcMotorEx.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        sleep(2000);

        // Offset
        dcMotorEx.setTargetPosition(tickOffset);
        dcMotorEx.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        dcMotorEx.setPower(1.0);
        while(dcMotorEx.isBusy());

        this.tickOffset = tickOffset;
    }

    public DcMotorEx getDcMotorEx() {
        return dcMotorEx;
    }
}
