package org.firstinspires.ftc.teamcode.util.info;

public class MotorInfo {
    public MotorInfo(double encoderResolutionPPR, double speedRpm, double torqueKgCm) {
        ENCODER_RESOLUTION_PPR = encoderResolutionPPR;
        SPEED_RPM = speedRpm;
        TORQUE_KG_CM = torqueKgCm;
    }

    public final double ENCODER_RESOLUTION_PPR;
    public final double SPEED_RPM;
    public final double TORQUE_KG_CM;
}
