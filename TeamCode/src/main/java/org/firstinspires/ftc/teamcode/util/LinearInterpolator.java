package org.firstinspires.ftc.teamcode.util;

/**
 * {@link LinearInterpolator} Has a function to interpolate data and estimate launch angle.
 * @author Jonny King
 */

public class LinearInterpolator {
    //These arrays contain data on various distances and the required angle of launch.
    private static Double[] dataPosition;
    private static Double[] dataAngle;

    public LinearInterpolator(Double[] positions, Double[] angles){
        dataPosition = positions;
        dataAngle = angles;
    }

    //This function returns angle of launch based on robot distance from goal.
    public static Double getAngle(Double distance){
        Double angle;
        int secondIndex = 0;

        for (int i = 0; i < dataPosition.length; i++){
            if (distance < dataPosition[i]){
                secondIndex = i;
                break;
            }
        }

        if (secondIndex == 0){
            return null;
        }

        angle = dataAngle[secondIndex - 1] + (dataAngle[secondIndex] - dataAngle[secondIndex - 1]) /
                (dataPosition[secondIndex] - dataPosition[secondIndex - 1]) *
                (distance - dataPosition[secondIndex - 1]);
        return angle;
    }
}
