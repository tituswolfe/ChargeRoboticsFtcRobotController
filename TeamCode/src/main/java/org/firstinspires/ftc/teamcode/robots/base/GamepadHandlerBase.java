package org.firstinspires.ftc.teamcode.robots.base;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.robots.base.opmodes.OpModeBase;

/**
 * {@link GamepadHandlerBase} contains base functions for gamepad handling.
 * {@link GamepadHandlerBase} takes generics for a subclasses of {@link RobotBase} and of {@link OpModeBase}.
 * @author Titus Wolfe
 */
public abstract class GamepadHandlerBase<Robot extends RobotBase, OpMode extends OpModeBase> {
    protected final Robot robot;
    protected final OpMode opMode;
    private final Gamepad gamepad;
    public boolean allowDrive;

    public static boolean isRobotCentric = false;                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         ;
    private long unlockTime = 0;
    public double speedFactor = 1.0;

    public static double slowSpeedFactor = 0.3;
    public static boolean isSlowMode = false;

    public GamepadHandlerBase(Robot robot, OpMode opMode, Gamepad gamepad, boolean allowDrive) {
        this.robot = robot;
        this.opMode = opMode;
        this.gamepad = gamepad;
        this.allowDrive = allowDrive;

    }

    public final void processGamepadControls() {
        if (opMode.getOpModeType() == OpModeBase.OpModeType.AUTO) return; // TODO: Consider adding init controls process

        if (allowDrive) processDriveControls(gamepad);
        if (unlockTime < System.currentTimeMillis()) onProcessGamepadControls(gamepad);
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

        if (gamepad.rightStickButtonWasPressed()) {
            isSlowMode = !isSlowMode;
            if (isSlowMode) {
                speedFactor = slowSpeedFactor;
            } else {
                speedFactor = 1.0;
            }
        }

        if (gamepad.leftStickButtonWasPressed()) {
            isRobotCentric = !isRobotCentric;
        }

        double xSpeed;
        double ySpeed;

        if (isRobotCentric) { // Robot Centric
            xSpeed = -gamepad.left_stick_y;
            ySpeed = -gamepad.left_stick_x;
        } else if(opMode.getAllianceColor() == OpModeBase.AllianceColor.RED) { // Field centric red alliance
            xSpeed = gamepad.left_stick_x;
            ySpeed = -gamepad.left_stick_y;
        } else if (robot.getFieldType() == RobotBase.FieldType.DIAMOND) { // Field centric blue alliance diamond field
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