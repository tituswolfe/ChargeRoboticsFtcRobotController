package org.firstinspires.ftc.teamcode.math.splines;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.math.Constants;

public class QunticHermiteSpline extends Spline {
    double x0;
    double x1;
    double dx0;
    double dx1;
    double y0;
    double y1;
    double dy0;
    double dy1;

    double ax;
    double bx;
    double cx;
    double ex;
    double fx;

    double ay;
    double by;
    double cy;
    double ey;
    double fy;


    public QunticHermiteSpline(Pose2D p0, Pose2D p1, double scaleFactor) {
        calculateCoefficients(p0, p1, scaleFactor);
        calculateLength(1000);
    }

    @Override
    void calculateCoefficients(Pose2D p0, Pose2D p1, double scaleFactor) {
        //double startX,
        // double startY,
        // double startAngle,
        // double endX,
        // double endY,
        // double endAngle,
        // scale factor


        x0 = p0.getX(Constants.standardDistanceUnit);
        x1 = p1.getX(Constants.standardDistanceUnit);

        dx0 = Math.cos(p0.getHeading(AngleUnit.RADIANS));
        dx1 = Math.cos(p1.getHeading(AngleUnit.RADIANS));

        y0 = p0.getY(Constants.standardDistanceUnit);
        y1 = p1.getY(Constants.standardDistanceUnit);

        dy0 = Math.sin(p0.getHeading(AngleUnit.RADIANS));
        dy1 = Math.sin(p1.getHeading(AngleUnit.RADIANS));

        double scale = scaleFactor * Math.sqrt(Math.pow(x1 - x0, 2) + Math.pow(y1 - y0, 2)); // 1 - 3 probs for scale fac

        dx0 *= scale;
        dx1 *= scale;
        dy0 *= scale;
        dy1 *= scale;

        ax = -6 * x0 - 3 * dx0 + 3 * dx1 + 6 * x1;
        bx = 15 * x0 + 8 * dx0 - 7 * dx1 - 15 * x1;
        cx = -10 * x0 - 6 * dx0 + 4 * dx1 + 10 * x1;
        ex = dx0;
        fx = x0;

        ay = -6 * y0 - 3 * dy0 + 3 * dy1 + 6 * y1;
        by = 15 * y0 + 8 * dy0 - 7 * dy1 - 15 * y1;
        cy = -10 * y0 - 6 * dy0 + 4 * dy1 + 10 * y1;
        ey = dy0;
        fy = y0;
    }

    @Override
    void calculateLength(int resolution) {
        double length = 0;
        Pose2D lastPose = evaluateAtNormalized((double) 0 / resolution);

        for (int i = 1; i < resolution; i++) {
            Pose2D nextPose = evaluateAtNormalized((double) i / resolution);

            length += Math.abs(Math.hypot(
                    nextPose.getX(Constants.standardDistanceUnit) - lastPose.getX(Constants.standardDistanceUnit),
                    nextPose.getY(Constants.standardDistanceUnit) - lastPose.getY(Constants.standardDistanceUnit)
            ));
        }

        setLength(length);
    }

    @Override
    public Pose2D evaluateAt(double targetLength) {
        return evaluateAtNormalized(targetLength / getLength());
    }

    @Override
    public Pose2D evaluateAtNormalized(double t) {
        if (t > 1) {
            t = 1;
        } else if (t < 0) {
            t = 0;
        }

        double x = ax * t * t * t * t * t + bx * t * t * t * t + cx * t * t * t + ex * t + fx;
        double y = ay * t * t * t * t * t + by * t * t * t * t + cy * t * t * t + ey * t + fy;
        double dx_dt = 5 * ax * t * t * t * t + 4 * bx * t * t * t + 3 * cx * t * t + ex;
        double dy_dt = 5 * ay * t * t * t * t + 4 * by * t * t * t + 3 * cy * t * t + ey;

        return new Pose2D(
                DistanceUnit.INCH,
                x,
                y,
                AngleUnit.RADIANS,
                Math.atan2(dx_dt, dy_dt)
        );
    }


    // change in gradiant????? potentaly . . .
}
