package org.firstinspires.ftc.teamcode.robots.season.decode.dugjetfire2;

import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robots.base.GamepadMapping;
import org.firstinspires.ftc.teamcode.robots.base.opmodes.TeleOpBase;
import org.firstinspires.ftc.teamcode.robots.season.decode.jetfire.JetFireGamepadMapping;
import org.firstinspires.ftc.teamcode.robots.season.decode.jetfire.JetFireRobot;


@TeleOp(name = "Jetfire2")
public class TestTeleop extends TeleOpBase<DugJetFire2Robot> {
    @Override
    public void buildPaths(Follower follower) {

    }



    @Override
    public void updateTelemetry(TelemetryManager telemetry) {


        super.updateTelemetry(telemetry);
    }

    @Override
    protected DugJetFire2Robot instantiateRobot() {
        return new DugJetFire2Robot();
    }

    @Override
    protected GamepadMapping<DugJetFire2Robot> instantiateGamepadMapping1() {
        return new GamepadMapping<DugJetFire2Robot>(robot) {
            @Override
            public void onYPressed() {

            }

            @Override
            public void onBPressed() {

            }

            @Override
            public void onAPressed() {

            }

            @Override
            public void onXPressed() {

            }

            @Override
            public void leftJoystick(float x, float y) {

            }

            @Override
            public void rightJoystick(float x, float y) {

            }

            @Override
            public void leftTrigger(float val) {

            }

            @Override
            public void rightTrigger(float val) {

            }

            @Override
            public void onLeftTriggerPressed() {

            }

            @Override
            public void onRightTriggerPressed() {

            }

            @Override
            public void onLeftTriggerReleased() {

            }

            @Override
            public void onRightTriggerReleased() {

            }

            @Override
            public void onLeftBumperPressed() {

            }

            @Override
            public void onRightBumperPressed() {

            }

            @Override
            public void onDpadUpPressed() {

            }

            @Override
            public void onDpadRightPressed() {

            }

            @Override
            public void onDpadDownPressed() {

            }

            @Override
            public void onDpadLeftPressed() {

            }
        };
    }

    @Override
    protected GamepadMapping<DugJetFire2Robot> instantiateGamepadMapping2() {
        return null; // new JetFireGamepadMapping2(robot);
    }
}
