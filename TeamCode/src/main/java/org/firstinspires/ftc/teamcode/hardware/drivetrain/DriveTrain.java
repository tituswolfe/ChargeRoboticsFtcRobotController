package org.firstinspires.ftc.teamcode.hardware.drivetrain;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.hardware.odometry.Odometry;
import org.firstinspires.ftc.teamcode.math.Constants;
import org.firstinspires.ftc.teamcode.math.splines.QunticHermiteSpline;
import org.firstinspires.ftc.teamcode.util.ThreadUtil;

//    MECANUM
//    X_DRIVE
//    KIWI
//    SWERVE,
//    DIFFRESIAL


//           case KIWI:
//                return new double[] {
//                        (-0.5 * x) - Math.sqrt(3)/2 * y + r,
//                        (-0.5 * x) + Math.sqrt(3)/2 * y + r,
//                        x + r
//                };
//                break;

public class DriveTrain {
    // Motors
    private final DcMotorEx frontLeft;
    private final DcMotorEx frontRight;
    private final DcMotorEx backLeft;
    private final DcMotorEx backRight;


    // Constants
    private final double wheelCircumference;
    private final double encoderPulsesPeRevolution;

    private final Odometry odometry;

    private PIDCoefficients driveCoefficients;
    private PIDCoefficients turnCoefficients;
    private double kdActivationDistance = 12; // within this distance, allowed to use kd
    private double kdMaxUseVelocity = 5; // as long as velocity greater than this, than kd

    private double positionThreshold = 0.25;
    private double headingThreshold = 0.25;

    private double minSpeed;
    private boolean isEnabled = true;

    // Stall detection
    ElapsedTime stallTime = new ElapsedTime();
    private boolean isAtStall = false;
    volatile boolean isStallDetectionEnabled = false;

    // Enums
    enum DriveMode {
        WAYPOINT,
        DRIVE_TO,
        HOLD_POSITION,
        PID_CONTROLLER_TESTING
    }

    public DriveTrain(
            DcMotorEx frontLeft,
            DcMotorEx frontRight,
            DcMotorEx backLeft,
            DcMotorEx backRight,
            Odometry odometry,
            PIDCoefficients driveCoefficients,
            PIDCoefficients turnCoefficients,
            DistanceUnit wheelDiameterUnit,
            double wheelDiameter,
            double encoderPulsesPeRevolution,
            double minSpeed
    ) {

        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.backLeft = backLeft;
        this.backRight = backRight;

        this.odometry = odometry;
        this.driveCoefficients = driveCoefficients;
        this.turnCoefficients = turnCoefficients;

        this.wheelCircumference = DistanceUnit.INCH.fromUnit(wheelDiameterUnit, wheelDiameter) * Math.PI;
        this.encoderPulsesPeRevolution = encoderPulsesPeRevolution;
        this.minSpeed = minSpeed;

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

//
//        frontLeft.setCurrentAlert(9.0, CurrentUnit.AMPS);
//        frontRight.setCurrentAlert(9.0, CurrentUnit.AMPS);
//        backRight.setCurrentAlert(9.0, CurrentUnit.AMPS);
//        backLeft.setCurrentAlert(9.0, CurrentUnit.AMPS);
    }


    public void driveTo(double targetX, double targetY, double targetHeading, double maxSpeed) {
        drive(
                new Pose2D(Constants.standardDistanceUnit, targetX, targetY, AngleUnit.DEGREES, targetHeading),
                DriveMode.DRIVE_TO,
                maxSpeed
        );
    }
    public boolean drive(Pose2D targetPose, DriveMode driveMode, double maxSpeed) {
        setMotorRunModes(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        if (maxSpeed < minSpeed) {
            maxSpeed = minSpeed;
        }

        //enableStallDetection();

        // Set stop conditions & actions

        QuadPredicate<Double, Double, Boolean, Double> stopCondition = null; // absoluteDistanceError, headingError, isAtStall, velocity
        Runnable stopAction = null;

        switch (driveMode) {
            case WAYPOINT:
                stopCondition = (absoluteDistanceError, headingError, isAtStall, velocity) -> absoluteDistanceError < followRadius  || isAtStall; // heading
                stopAction = () -> {};
                break;
            case DRIVE_TO:
                stopCondition = (absoluteDistanceError, headingError, isAtStall, velocity) -> absoluteDistanceError < positionThreshold && Math.abs(headingError) < headingThreshold && velocity < 6;
                stopAction = () -> {
                    stopMotors(DcMotor.ZeroPowerBehavior.BRAKE);
                    setMotorRunModes(DcMotor.RunMode.RUN_USING_ENCODER);
                };
                break;
            case HOLD_POSITION:

                stopAction = () -> {};
                break;
            case PID_CONTROLLER_TESTING:
                stopCondition = (absoluteDistanceError, headingError, isAtStall, velocity) -> Math.abs(absoluteDistanceError) < followRadius && Math.abs(headingError) < headingThreshold || isAtStall && velocity < 10;
                stopAction = () -> {};
                break;
        }

        assert stopCondition != null;



        boolean isActive = true;
        // Drive until stop condition is met
        while(isActive && isEnabled) {
            odometry.update();

            double xDistanceError = targetPose.getX(DistanceUnit.INCH) - odometry.getXPos(DistanceUnit.INCH);
            double yDistanceError = targetPose.getY(DistanceUnit.INCH) - odometry.getYPos(DistanceUnit.INCH);
            double absoluteDistanceError = Math.abs(Math.hypot(xDistanceError, yDistanceError));

            double xVelocity = Math.abs(odometry.getXVel(DistanceUnit.INCH));
            double yVelocity = Math.abs(odometry.getYVel(DistanceUnit.INCH));
            double velocity = Math.abs(Math.hypot(xVelocity, yVelocity));

            double headingError = calculateHeadingError(
                    Odometry.convertAngleSystem(targetPose.getHeading(AngleUnit.DEGREES), AngleUnit.DEGREES, Odometry.AngleSystem.UNSIGNED),
                    odometry.getHeading(AngleUnit.DEGREES, Odometry.AngleSystem.UNSIGNED)
            );

            double headingVelocity = odometry.getHeadingVel(AngleUnit.DEGREES);

            double xSpeed = clampRange(
                    calculatePidOutput(
                            xDistanceError,
                            0,
                            (Math.abs(xDistanceError) < kdActivationDistance && xVelocity > kdMaxUseVelocity) ? xVelocity : 0,
                            driveCoefficients
                    ),
                    minSpeed,
                    maxSpeed,
                    true
            );

            double ySpeed = clampRange(
                    calculatePidOutput(
                            yDistanceError,
                            0,
                            (Math.abs(yDistanceError) < kdActivationDistance && yVelocity > kdMaxUseVelocity) ? yVelocity : 0,
                            driveCoefficients
                    ),
                    minSpeed,
                    maxSpeed,
                    true
            );

            double rSpeed = clampRange(
                    calculatePidOutput(
                            headingError,
                            0,
                            (Math.abs(headingVelocity) < kdActivationDistance && headingVelocity > kdMaxUseVelocity) ? -headingVelocity : 0,
                            turnCoefficients
                    ),
                    minSpeed,
                    maxSpeed,
                    true
            );

            double sinHeading = getSinHeading();
            double cosHeading = getCosHeading();

//            double robotStrafeSpeed = xSpeed * cosHeading + ySpeed * sinHeading;
//            double robotForwardSpeed = ySpeed * cosHeading - xSpeed * sinHeading;

            double robotStrafeSpeed = calculateRobotStrafeFromFieldVelocity(xSpeed, cosHeading, ySpeed, sinHeading);
            double robotForwardSpeed = calculateRobotForwardFromFieldVelocity(xSpeed, cosHeading, ySpeed, sinHeading);



            setMotorPowersThroughKinematicTransformation(robotStrafeSpeed, robotForwardSpeed, rSpeed);

            if (stopCondition.test(absoluteDistanceError, headingError, isAtStall, velocity)) {
                isActive = false;
                stopAction.run();
            }
        }

        disableStallDetection();

        return false;
    }

    public double calculateRobotStrafeFromFieldVelocity(double xVel, double cosHeading, double yVel, double sinHeading) {
        return xVel * cosHeading + yVel * sinHeading;
    }

    public double calculateRobotForwardFromFieldVelocity(double xVel, double cosHeading, double yVel, double sinHeading) {
        return yVel * cosHeading - xVel * sinHeading;
    }

    public double getSinHeading() {
        return Math.sin(odometry.getHeading(AngleUnit.RADIANS, Odometry.AngleSystem.SIGNED));
    }

    public double getCosHeading() {
        return Math.cos(odometry.getHeading(AngleUnit.RADIANS, Odometry.AngleSystem.SIGNED));
    }



    public static double calculateHeadingError(double currentHeading, double targetHeading) {
        double error = currentHeading - targetHeading;

        if (Math.abs(error) > 180) {
            if (error > 0) {
                error -= 360;
            } else {
                error += 360;
            }
        }

        return error;
    }

    //public void driveTo(Pose2D targetPose, HolonomicDriveTrain.DriveMode driveMode, double maxSpeed, double minSpeed, boolean breakMotors) {

//        enum Direction {
//            NORTH,
//            NORTH_EAST,
//            EAST,
//            SOUTH_EAST,
//            SOUTH,
//            SOUTH_WEST,
//            WEST,
//            NORTH_WEST,
//
//        }

    double followRadius = 1.0;







    public void splineTo(Pose2D targetPose) {
        QunticHermiteSpline qunticHermiteSpline = new QunticHermiteSpline(odometry.getPose2D(), targetPose, 1.5);
        for(int t = 0; t <= qunticHermiteSpline.getLength(); t += 1) {
            drive(qunticHermiteSpline.evaluateAt(t), DriveMode.WAYPOINT, 0.4);
        }

        //drive(targetPose, DriveMode.DRIVE_TO, 1.0);
    }

    //  boolean alignWithTangent, double oftset

    // double heading ofset



    @FunctionalInterface
    public interface QuadPredicate<T, U, V, W> {
        boolean test(T t, U u, V v, W w);
    }

    public void enableStallDetection() {
        ThreadUtil.runAsync(() -> {
            boolean isMoving = false;
            while(isStallDetectionEnabled) {
                if (Math.hypot(odometry.getXVel(DistanceUnit.INCH), odometry.getYVel(DistanceUnit.INCH)) > 3 && getHighestMotorSpeed() > 0) {
                    isMoving = true;
                    continue;
                }

                if (isMoving) {
                    isMoving = false;
                    stallTime.reset();
                } else if (stallTime.milliseconds() > 300) {
                    isAtStall = true;
                }
            }
            Thread.currentThread().interrupt();
        });
    }

    public void disableStallDetection() {
        isStallDetectionEnabled = false;
        isAtStall = false;
    }

    public boolean isAtStall() {
        return isAtStall;
    }

    public void setStall(boolean isAtStall) {
        this.isAtStall = isAtStall;
    }


    public static double calculatePidOutput(double proportionalError, double integralError, double derivativeError, PIDCoefficients pidCoefficients) {
        return (pidCoefficients.p * proportionalError) + (pidCoefficients.i * integralError) + (pidCoefficients.d * derivativeError);
    }



    public void setMotorPowersThroughKinematicTransformation(double x, double y, double r) {
        double frontLeftPower = +y + x - r;
        double frontRightPower = y - x + r;
        double backLeftPower = y - x - r;
        double backRightPower = +y + x + r;

        // Proportionally adjust power to not exceed 1.0
        double max = Math.max(
                1.0,
                Math.max(Math.abs(frontLeftPower),
                        Math.max(Math.abs(frontRightPower),
                                Math.max(Math.abs(backLeftPower),
                                        Math.abs(backRightPower)
                                )
                        )
                )
        );

        frontLeftPower /= max;
        frontRightPower /= max;
        backLeftPower /= max;
        backRightPower /= max;

        setMotorPowers(frontLeftPower, frontRightPower, backLeftPower, backRightPower);
    }


    public void tankDrive(double distance, double power) {
        addToTargetPositions(0,0,0,0);
        setMotorRunModes(DcMotorEx.RunMode.RUN_TO_POSITION);
        int encoderPulses =  (int) (((distance / wheelCircumference) * encoderPulsesPeRevolution));
        addToTargetPositions(encoderPulses, encoderPulses, encoderPulses, encoderPulses);
        setMotorPowers(Math.abs(power));
        waitUntilMotorsStop();
    }


    public void strafeDrive(double distance, double power) {
        addToTargetPositions(0,0,0,0);
        setMotorRunModes(DcMotorEx.RunMode.RUN_TO_POSITION);
        int encoderPulses =  (int) (((distance / wheelCircumference) * encoderPulsesPeRevolution));
        addToTargetPositions(encoderPulses, -encoderPulses, encoderPulses, -encoderPulses);
        setMotorPowers(Math.abs(power));
        waitUntilMotorsStop();
    }

    public void turn(double targetHeading, double headingThreshold) {
        // TODO: Call drive func
    }

//    public void turn(double targetHeading, double headingThreshold, double velocityThreshold, double maxPower, double minPower) {
//        setMotorRunModes(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        boolean isFirstCalculation = true;
//        double headingError = 0;
//        double headingVelocity = 0;
//        double kpTurnSpeed;
//        double kdTurnSpeed;
//        double rSpeed;
//
//        while(Math.abs(headingError) > headingThreshold || Math.abs(headingVelocity) > velocityThreshold || isFirstCalculation && isEnabled) {
//            odometry.update();
//
//            headingError = AngleUnit.normalizeDegrees(targetHeading - odometry.getHeading(AngleUnit.DEGREES, false));
//            headingVelocity = odometry.getHeadingVel(AngleUnit.DEGREES);
//
//            kpTurnSpeed = headingError * turnCoefficients.p;
//            kdTurnSpeed = (Math.abs(kpTurnSpeed) > 1) ? 0 : headingVelocity * turnCoefficients.d;
//            rSpeed = (Math.abs(headingError) > headingThreshold) ? kpTurnSpeed - kdTurnSpeed : 0.0;
//            if (rSpeed != 0) {
//                rSpeed = clampRange(rSpeed, minPower, maxPower, true);
//            }
//
//            setMotorPowersThroughKinematicTransformation(0, 0, rSpeed);
//            isFirstCalculation = false;
//        }
//        stopMotors();
//    }

    public double clampRange(double value, double min, double max, boolean preserveSign) {
        min = Math.abs(min);
        max = Math.abs(max);

        if (preserveSign) {
            if (Math.abs(value) < min) {
                return min * Math.signum(value);
            } else if (Math.abs(value) > max) {
                return max * Math.signum(value);
            }
            return value;
        } else {
            return Math.max(min, Math.min(max, value));
        }
    }



    public void waitUntilMotorsStop() {
        while(frontLeft.isBusy() || frontRight.isBusy() || backLeft.isBusy() || backRight.isBusy() && isEnabled);
//        if (!isEnabled) {
//            stopMotors();
//            addToTargetPositions(0 ,0 ,0 ,0);
//            setMotorRunModes(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        }
    }


    public void setMotorPowers(double frontLeftPower, double frontRightPower, double backLeftPower, double backRightPower) {
        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
        backLeft.setPower(backLeftPower);
        backRight.setPower(backRightPower);
    }

    public void stopMotors(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        setMotorPowers(0, 0, 0, 0);
        setZeroPowerBehaviors(zeroPowerBehavior);
    }

    public void setMotorPowers(double power) {
        frontLeft.setPower(power);
        frontRight.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(power);
    }

    public void setMotorRunModes(DcMotor.RunMode runMode){
        frontLeft.setMode(runMode);
        frontRight.setMode(runMode);
        backLeft.setMode(runMode);
        backRight.setMode(runMode);
    }

    public void setZeroPowerBehaviors(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        frontLeft.setZeroPowerBehavior(zeroPowerBehavior);
        frontRight.setZeroPowerBehavior(zeroPowerBehavior);
        backLeft.setZeroPowerBehavior(zeroPowerBehavior);
        backRight.setZeroPowerBehavior(zeroPowerBehavior);
    }


    public void addToTargetPositions(int frontLeftAddend, int frontRightAddend, int backLeftAddend, int backRightAddend) {
        frontLeft.setTargetPosition(frontLeft.getCurrentPosition() + frontLeftAddend);
        frontRight.setTargetPosition(frontRight.getCurrentPosition() + frontLeftAddend);
        backLeft.setTargetPosition(backLeft.getCurrentPosition() + frontLeftAddend);
        backRight.setTargetPosition(backRight.getCurrentPosition() + frontLeftAddend);
    }

    public double getHighestMotorSpeed() {
        return Math.max(
                Math.abs(frontLeft.getPower()), Math.max(
                        Math.abs(frontRight.getPower()), Math.max(
                                Math.abs(backLeft.getPower()),
                                Math.abs(backRight.getPower())
                        )
                )
        );
    }

    public void enable() {
        isEnabled = true;
    }

    public void disable() {
        isEnabled = false;
    }


    public PIDCoefficients getDriveCoefficients() {
        return driveCoefficients;
    }

    public void setDriveCoefficients(PIDCoefficients driveCoefficients) {
        this.driveCoefficients = driveCoefficients;
    }

    public PIDCoefficients getTurnCoefficients() {
        return turnCoefficients;
    }

    public void setTurnCoefficients(PIDCoefficients turnCoefficients) {
        this.turnCoefficients = turnCoefficients;
    }

    public double getMinSpeed() {
        return minSpeed;
    }

    public void setMinSpeed(double minSpeed) {
        this.minSpeed = minSpeed;
    }
}
