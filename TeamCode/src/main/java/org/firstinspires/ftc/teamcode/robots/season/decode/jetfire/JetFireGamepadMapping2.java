package org.firstinspires.ftc.teamcode.robots.season.decode.jetfire;

public class JetFireGamepadMapping2 extends JetFireGamepadMapping {
    public JetFireGamepadMapping2(JetFireRobot decodeRobot) {
        super(decodeRobot);
    }

    @Override
    public void onLeftStickPressed() {
        super.onRightStickPressed();
    }

    @Override
    public void onRightStickPressed() {
        super.onLeftStickPressed();
    }
}
