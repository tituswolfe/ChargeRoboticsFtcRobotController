package org.firstinspires.ftc.teamcode.robots.base;

import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.robots.base.opmodes.BaseOpMode;

/**
 * {@link GamepadHandlerBase} contains base functions for gamepad handling.
 * {@link GamepadHandlerBase} takes generics for a subclasses of {@link RobotBase} and of {@link BaseOpMode}.
 * @author Titus Wolfe
 */
public abstract class GamepadHandlerBase<Robot extends RobotBase, OpMode extends BaseOpMode> {
    protected final Robot robot;
    protected final OpMode opMode;
    private final Gamepad gamepad;
    public boolean allowDrive;

    public boolean isRobotCentric = true;
    private long unlockTime = 0; // System time (in mills) when controls can unlock


    public GamepadHandlerBase(Robot robot, OpMode opMode, Gamepad gamepad, boolean allowDrive) {
        this.robot = robot;
        this.opMode = opMode;
        this.gamepad = gamepad;
        this.allowDrive = allowDrive;
    }

    public final void processGamepadControls() {
        if (allowDrive) {
            processDriveControls(gamepad);
        }

        if(unlockTime < System.currentTimeMillis()) {
            onProcessGamepadControls(gamepad);
        }
    }

    protected abstract void onProcessGamepadControls(Gamepad gamepad);

    public final void lockOutGamepad() {
        lockOutGamepad(150);
    }

    public final void lockOutGamepad(int mills) {
        unlockTime = System.currentTimeMillis() + mills;
    }

    // TODO: Test with Pedro Pathing field centric
    public final void processDriveControls(Gamepad gamepad) {
        robot.getFollower().setTeleOpDrive(gamepad.left_stick_x, gamepad.left_stick_y, gamepad.right_stick_x, true);
//        if (isRobotCentric) {
//            robot.getFollower().setTeleOpDrive(gamepad.left_stick_y, gamepad.left_stick_x, gamepad.right_stick_x, true);
//        } else {
//            processFieldCentricDrive();
//        }
    }

    public final void processFieldCentricDrive() {
        if (opMode.getFieldType() != BaseOpMode.FieldType.DIAMOND) {
            if (opMode.getAllianceColor() == BaseOpMode.AllianceColor.RED) {
                robot.getFollower().setTeleOpDrive(gamepad.left_stick_x, gamepad.left_stick_y, gamepad.right_stick_x, true);
            } else {
                robot.getFollower().setTeleOpDrive(-gamepad.left_stick_x, -gamepad.left_stick_y, gamepad.right_stick_x, true);
            }
        } else {
            if (opMode.getAllianceColor() == BaseOpMode.AllianceColor.RED) {
                robot.getFollower().setTeleOpDrive(gamepad.left_stick_y, gamepad.left_stick_x, gamepad.right_stick_x, true);
            } else {
                robot.getFollower().setTeleOpDrive(gamepad.left_stick_x, -gamepad.left_stick_y, gamepad.right_stick_x, true);
            }
        }
    }
}