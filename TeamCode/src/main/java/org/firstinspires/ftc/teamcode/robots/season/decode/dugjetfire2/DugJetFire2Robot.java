package org.firstinspires.ftc.teamcode.robots.season.decode.dugjetfire2;

import static org.firstinspires.ftc.teamcode.hardware.drivetrain.pedroPathing.Constants.createDecodeFollower;
import static org.firstinspires.ftc.teamcode.hardware.drivetrain.pedroPathing.Constants.createFollower;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.control.PIDFController;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.hardware.controllers.motor.RPSController;
import org.firstinspires.ftc.teamcode.hardware.controllers.servo.RGBIndicatorLightController;
import org.firstinspires.ftc.teamcode.hardware.controllers.servo.ServoTimerController;
import org.firstinspires.ftc.teamcode.robots.base.RobotBase;
import org.firstinspires.ftc.teamcode.robots.base.StaticData;
import org.firstinspires.ftc.teamcode.robots.base.opmodes.OpModeBase;
import org.firstinspires.ftc.teamcode.util.LinearInterpolator;
import org.firstinspires.ftc.teamcode.util.LogicUtil;

import java.util.TreeMap;

@Configurable
public class DugJetFire2Robot extends RobotBase {

    @Override
    public void initHardware(HardwareMap hardwareMap) {

    }

    @Override
    public void startConfiguration() {

    }

    @Override
    protected FieldType instantiateFieldType() {
        return null;
    }

    @Override
    public Follower instantiateFollower(HardwareMap hardwareMap) {
        return createDecodeFollower(hardwareMap);
    }

    @Override
    public Limelight3A instantiateLimelight3A(HardwareMap hardwareMap) {
        return null;
    }
}
