package org.firstinspires.ftc.teamcode.util;
/*
 * {@link BestFitPolynomial} Calculates the coefficients of the best-fit polynomial
 * of an input degree for the data. It also contains a function that uses the coefficients
 * to calculate the estimated launch angle given a certain distance.
 *
 * @author Jonny King
 */
public class BestFitPolynomial {
    private static double[] dataPosition;
    private static double[] dataAngle;
    private static double[] coefficients;

    //The mega-constructor
    public BestFitPolynomial(int degree, double[] positions, double[] angles){
        dataPosition = positions;
        dataAngle = angles;

        //Calculate integer powers of all the positions
        double[][] positionPowers = new double[2 * degree + 1][dataPosition.length];
        for (int i = 0; i < dataPosition.length; i++){
            positionPowers[0][i] = 1.0;
        }
        for (int i = 1; i <= 2 * degree; i++){
            for (int j = 0; j < dataPosition.length; j++){
                positionPowers[i][j] = positionPowers[i - 1][j] * dataPosition[j];
            }
        }

        //Sum all numbers of a certain power
        double[] posPowSums = new double[2 * degree + 1];
        posPowSums[0] = dataPosition.length;
        for (int i = 1; i <= 2 * degree; i++){
            posPowSums[i] = 0;
            for (int j = 0; j < dataPosition.length; j++){
                posPowSums[i] += positionPowers[i][j];
            }
        }

        //Create coefficient matrix
        double[][] matrix = new double[degree + 1][degree + 1];
        for (int i = 0; i <= degree; i++){
            for(int j = 0; j <= degree; j++){
                matrix[i][j] = posPowSums[degree + i - j];
            }
        }

        //Create matrix augment
        double[] augment = new double[degree + 1];
        for (int i = 0; i <= degree; i++){
            augment[i] = 0;
            for (int j = 0; j <= degree; j++){
                augment[i] += dataAngle[j] * positionPowers[i][j];
            }
        }

        //Bring matrix to REF and change augment accordingly
        for (int i = 1; i <= degree; i++){
            matrix[0][i] /= matrix[0][0];
        }
        augment[0] /= matrix[0][0];
        matrix[0][0] = 1;
        for (int i = 1; i <= degree; i++){
            for (int j = 0; j < i; j++){
                for (int k = 0; k <= degree; k++){
                    matrix[i][k] -= matrix[j][k] * matrix[i][j];
                }
                augment[i] -= augment[j] * matrix[i][j];
            }

            for (int j = i + 1; j <= degree; j++){
                matrix[i][j] /= matrix[i][i];
            }
            augment[i] /= matrix[i][i];
            matrix[i][i] = 1;
        }

        //Bring matrix to RREF and solve
        for (int i = degree - 1; i >= 0; i--){
            for (int j = i + 1; j <= degree; j++){
                augment[i] -= augment[j] * matrix[i][j];
            }
        }

        coefficients = augment;
    }

    //The function that actually uses the polynomial coefficients to estimate the angle
    public static double getAngle(double distance){
        double angle = coefficients[0];
        for (int i = 1; i < coefficients.length; i++){
            angle *= distance;
            angle += coefficients[i];
        }
        return angle;
    }
}
