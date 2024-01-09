// Copyright 2019 FRC Team 3476 Code Orange

package frc.robot.lib;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.geometry.Twist3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class Utility {
    public static final Rotation2d R2D_IDENTITY = new Rotation2d();
    public static final Translation2d TR2D_IDENTITY = new Translation2d();
    public static final ChassisSpeeds CS_IDENTITY = new ChassisSpeeds();
    public static final Twist2d TW2D_IDENTITY = new Twist2d();
    public static final Rotation3d R3D_IDENTITY = new Rotation3d();
    public static final Translation3d TR3D_IDENTITY = new Translation3d();
    public static final Twist3d TW3D_IDENTITY = new Twist3d();
    public static final Pose2d P2D_IDENTITY = new Pose2d();
    public static final Pose3d P3D_IDENTITY = new Pose3d();

    /**
     * Keeps a value in a range by truncating it.
     *
     * @param toCoerce the value to coerce
     * @param high     the high value of the range
     * @param low      the low value of the range
     * @return the coerced value
     */
    public static double coerce(double toCoerce, double high, double low) {
        if (toCoerce > high) {
            return high;
        } else if (toCoerce < low) {
            return low;
        }
        return toCoerce;
    }

    /**
     * Donuts an input value so that a control loop can overcome backlash or friction.
     *
     * @param toDonut   the input value
     * @param threshold the backlash or friction scalar
     * @return the adjusted "input" value for evaluation by the control loop
     */
    public static double donut(double toDonut, double threshold) {
        if (toDonut == 0) {
            return 0;
        }
        if (toDonut > 0) {
            return toDonut + threshold;
        }
        return toDonut - threshold;
    }

    public static double getArrayMax(double[] array) {
        double result = array[0];
        for (double e : array) {
            if (e > result) {
                result = e;
            }
        }
        return result;
    }

    public static double getArrayMin(double[] array) {
        double result = array[0];
        for (double e : array) {
            if (e < result) {
                result = e;
            }
        }
        return result;
    }

    public static boolean inRange(double test, double a, double b) {
        return Utility.inRange(test, new double[]{a, b});
    }

    public static boolean inRange(double test, double[] range) {
        return test > Utility.getArrayMin(range) && test < Utility.getArrayMax(range);
    }

    public static double normalize(double toNormalize, double fromHigh, double fromLow, double toHigh, double toLow) {
        double factor = (toHigh - toLow) / (fromHigh - fromLow);
        double add = toLow - fromLow * factor;
        return toNormalize * factor + add;
    }

    public static double coercedNormalize(double rawValue, double minInput, double maxInput, double minOutput,
                                          double maxOutput) {
        if (rawValue < minInput) {
            return minOutput;
        } else if (rawValue > maxInput) {
            return maxOutput;
        }
        double norm = (Math.abs(rawValue) - minInput) / (maxInput - minInput);
        norm = Math.copySign(norm * (maxOutput - minOutput), rawValue) + minOutput;
        return norm;
    }

    public static double getSpeedAsScalar(ChassisSpeeds speeds) {
        // why is this not a WPILib feature yet?
        return new Translation2d(speeds.vxMetersPerSecond, speeds.vxMetersPerSecond).getNorm();
    }

    public static double getLerpT(double query, double low, double high) {
        return (query - low) / (high - low);
    }

    public static double interpolateLinear(double queryX, double lowX, double highX, double lowY, double highY) {
        return MathUtil.interpolate(lowY, highY, getLerpT(queryX, lowX, highX));
    }

    public static double interpolateLagrange(double queryX, Translation2d... knownPoints) {
        double result = 0;
        for (int i = 0; i < knownPoints.length; i++) { // Goes through points.
            double term = knownPoints[i].getY(); // Y-value of selected point.
            for (int j = 0; j < knownPoints.length; j++) { // Loops through non-identical points.
                if (j != i) { // Avoids multiplication by 0.
                    // Interpolates between selected and point from data set.
                    term *= (queryX - knownPoints[j].getX()) / (knownPoints[i].getX() - knownPoints[j].getX());
                }
            }
            result += term; // Accumulated interpretation is added.
        }
        return result;
    }

    public static Rotation2d rotationModulus(Rotation2d input) {
        return Rotation2d.fromRotations(MathUtil.inputModulus(input.getRotations(), 0, 1));
    }

    public static double sum(double[] arr) {
        double sum = 0;
        for (double d : arr) {
            sum += d;
        }
        return sum;
    }

    public static Twist2d getTwist2dFromChassisSpeeds(ChassisSpeeds speeds) {
        return new Twist2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond);
    }
}
