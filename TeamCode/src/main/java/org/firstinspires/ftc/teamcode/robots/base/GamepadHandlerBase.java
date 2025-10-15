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

    public boolean isRobotCentric = false                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    ;
    private long unlockTime = 0; // System time (in mills) when controls can unlock
    public double speedFactor = 0.7;

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

    public final void processDriveControls(Gamepad gamepad) {
        // Re-start teleop drive after path finished
        if (!robot.getFollower().isBusy() && !robot.getFollower().isTeleopDrive()) {
            robot.getFollower().startTeleopDrive();
        }

        if (isRobotCentric) {
            robot.getFollower().setTeleOpDrive(-gamepad.left_stick_y * speedFactor, -gamepad.left_stick_x * speedFactor, -gamepad.right_stick_x * speedFactor, true);
        } else {
            //robot.getFollower().setTeleOpDrive(gamepad.left_stick_y * speedFactor, gamepad.left_stick_x * speedFactor, -gamepad.right_stick_x * speedFactor, false);
            processFieldCentricDrive();
        }
    }

    // x y head for teleop drive field centric
    // left y is up and down, left x is right and left
    // right stick x is left and right

    public final void processFieldCentricDrive() {
        double xSpeed = 0;
        double ySpeed = 0;

        if(opMode.getAllianceColor() == BaseOpMode.AllianceColor.RED) {
            xSpeed = gamepad.left_stick_x;
            ySpeed = -gamepad.left_stick_y;
        } else {
            switch (opMode.getFieldType()) {
                case SQUARE:
                    xSpeed = -gamepad.left_stick_x;
                    ySpeed = gamepad.left_stick_y;
                    break;
                case SQUARE_INVERTED_ALLIANCE:
                    xSpeed = -gamepad.left_stick_x;
                    ySpeed = gamepad.left_stick_y;
                    break;
                case DIAMOND:
                    xSpeed = gamepad.left_stick_y;
                    ySpeed = gamepad.left_stick_x;
                    break;
            }
        }

        robot.getFollower().getVectorCalculator().setTeleOpMovementVectors(xSpeed * speedFactor, ySpeed * speedFactor, -gamepad.right_stick_x * speedFactor, false);
    }
}