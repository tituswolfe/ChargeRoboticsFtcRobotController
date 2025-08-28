package org.firstinspires.ftc.teamcode.hardware.odometry;


import static org.firstinspires.ftc.teamcode.util.ThreadUtil.sleep;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class GoBildaOdometry extends Odometry {
    private final GoBildaPinpointDriver goBildaPinpointDriver;
    
    public GoBildaOdometry(GoBildaPinpointDriver goBildaPinpointDriver, GoBildaPinpointDriver.EncoderDirection xEncoderDirection, GoBildaPinpointDriver.EncoderDirection yEncoderDirection) {
        this.goBildaPinpointDriver = goBildaPinpointDriver;

        goBildaPinpointDriver.setEncoderDirections(yEncoderDirection, xEncoderDirection);
        goBildaPinpointDriver.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        goBildaPinpointDriver.setOffsets(-75, -155);
    }

    @Override
    public double getXPos(DistanceUnit distanceUnit) {
        return -(distanceUnit.fromUnit(DistanceUnit.MM, goBildaPinpointDriver.getPosY()));
    }

    @Override
    public double getYPos(DistanceUnit distanceUnit) {
        return distanceUnit.fromUnit(DistanceUnit.MM, goBildaPinpointDriver.getPosX());
    }

    @Override
    public double getHeading(AngleUnit angleUnit, AngleSystem angleSystem) {
        return angleUnit.fromUnit(AngleUnit.RADIANS, convertAngleSystem(goBildaPinpointDriver.getHeading(), AngleUnit.RADIANS, angleSystem));
    }


    @Override
    public double getXVel(DistanceUnit distanceUnit) {
        return -(distanceUnit.fromUnit(DistanceUnit.MM, goBildaPinpointDriver.getVelY()));
    }

    @Override
    public double getYVel(DistanceUnit distanceUnit) {
        return distanceUnit.fromUnit(DistanceUnit.MM, goBildaPinpointDriver.getVelX());
    }

    @Override
    public double getHeadingVel(AngleUnit angleUnit) {
        return angleUnit.fromUnit(AngleUnit.RADIANS, goBildaPinpointDriver.getHeadingVelocity());
    }


    @Override
    public Pose2D getPose2D() { // swap x and y
        return new Pose2D(DistanceUnit.MM, getXPos(DistanceUnit.MM), getYPos(DistanceUnit.MM), AngleUnit.RADIANS, getHeading(AngleUnit.RADIANS, AngleSystem.UNSIGNED));
    }

    @Override
    public boolean setPose2D(Pose2D pose2D) {
        goBildaPinpointDriver.setPosition(pose2D);
        return true;
    }

    @Override
    public boolean update() {
        goBildaPinpointDriver.update();
        return true;
    }

    @Override
    public boolean setPosition(Pose2D pose2D) {
        goBildaPinpointDriver.setPosition(pose2D);
        return true;
    }

    @Override
    public boolean resetPos() {
        goBildaPinpointDriver.update();
        goBildaPinpointDriver.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.RADIANS, goBildaPinpointDriver.getHeading()));
        return true;
    }

    @Override
    public boolean resetHeading() {
        goBildaPinpointDriver.recalibrateIMU();
        sleep(270);
        return true;
    }

    @Override
    public boolean resetPosAndHeading() {
        goBildaPinpointDriver.resetPosAndIMU();
        sleep(270);
        return true;
    }
}
