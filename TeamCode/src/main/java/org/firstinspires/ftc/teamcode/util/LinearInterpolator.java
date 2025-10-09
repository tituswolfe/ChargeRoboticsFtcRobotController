package org.firstinspires.ftc.teamcode.util;

public class LinearInterpolator {
    //These arrays contain data on various distances and the required angle of launch.
    private static Double[] dataPosition = new Double[]{
           1.,2.,3.
    };
    private static Double[] dataAngle = new Double[]{
         2.,4.,6.
    };

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
