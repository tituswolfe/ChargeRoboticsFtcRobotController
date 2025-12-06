package org.firstinspires.ftc.teamcode.robots.season.decode.dug;

import static org.firstinspires.ftc.teamcode.hardware.drivetrain.pedroPathing.Constants.createDecodeFollower;

import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardware.controllers.motor.AngleMotorController;
import org.firstinspires.ftc.teamcode.hardware.controllers.motor.VelocityMotorController;
import org.firstinspires.ftc.teamcode.hardware.controllers.subsystems.Turret;
import org.firstinspires.ftc.teamcode.robots.base.RobotBase;
import org.firstinspires.ftc.teamcode.util.math.Angle;

public class DugRobot extends RobotBase {
    Turret turret;

    @Override
    public void initHardware(HardwareMap hardwareMap) {
//        turret = new Turret(
//                new VelocityMotorController(hardwareMap.get(DcMotorEx.class, "flywheel"), 28, 1),
//                new AngleMotorController(
//                        hardwareMap.get(DcMotorEx.class, "turntable"),
//                        384.5,
//                        ((double) 16 / 45),
//                        new Angle(120, false),
//                        new Angle(-120, false)
//                )
//        );
    }

    @Override
    public void startConfiguration() {

    }

    @Override
    public void updateHardwareStates() {

        super.updateHardwareStates();
    }

    @Override
    protected FieldType instantiateFieldType() {
        return FieldType.SQUARE_INVERTED_ALLIANCE;
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
