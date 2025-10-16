package org.firstinspires.ftc.teamcode.robots.base;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.robots.base.opmodes.BaseOpMode;

/**
 * {@link GamepadHandlerBase} contains base functions for gamepad handling.
 * {@link GamepadHandlerBase} takes generics for a subclasses of {@link RobotBase} and of {@link BaseOpMode}.
 * @author Titus Wolfe
 */
@Configurable
public abstract class GamepadHandlerBase<Robot extends RobotBase, OpMode extends BaseOpMode> {
    protected final Robot robot;
    protected final OpMode opMode;
    private final Gamepad gamepad;
    public boolean allowDrive;

    public static boolean isRobotCentric = false;                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         ;
    private long unlockTime = 0; // System time (in mills) when controls can unlock
    public double speedFactor = 0.7;

    public static boolean isFastSpeed = true;
    public static double fastSpeed = 1;
    public static double slowSpeed = 0.7;

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
        // TODO: init controls
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

        // TODO: Account for auto

        if (gamepad.rightStickButtonWasPressed()) {
            isFastSpeed = !isFastSpeed;
            if (isFastSpeed) {
                speedFactor = fastSpeed;
            } else {
                speedFactor = slowSpeed;
            }
        }

        if (gamepad.leftStickButtonWasPressed()) {
            isRobotCentric = !isRobotCentric;
        }

        double xSpeed = 0;
        double ySpeed = 0;

        if (isRobotCentric) { // Robot Centric
            xSpeed = -gamepad.left_stick_y;
            ySpeed = -gamepad.left_stick_x;
        } else if(opMode.getAllianceColor() == BaseOpMode.AllianceColor.RED) { // Field centric red alliance
            xSpeed = gamepad.left_stick_x;
            ySpeed = -gamepad.left_stick_y;
        } else if (opMode.getFieldType() == BaseOpMode.FieldType.DIAMOND) { // Field centric blue alliance diamond field
            xSpeed = gamepad.left_stick_y;
            ySpeed = gamepad.left_stick_x;
        } else { // Field centric blue alliance square & square inverted alliance
            xSpeed = -gamepad.left_stick_x;
            ySpeed = gamepad.left_stick_y;
        }

        robot.getFollower().getVectorCalculator().setTeleOpMovementVectors(xSpeed * speedFactor, ySpeed * speedFactor, -gamepad.right_stick_x * speedFactor, isRobotCentric);
    }
}

// JOYSTICK INPUT
//           | <---- y axis
//           |
//         -1.0
//           /\
//           ___
//         / \|/ \
// -1.0 < |--( )--| > 1.0 ---- x axis
//         \ /|\ /
//           ---
//           \/
//           1.0

// RIGHT STICK 0.0 to -1.0
// LEFT STICK 0.0 to 1.0