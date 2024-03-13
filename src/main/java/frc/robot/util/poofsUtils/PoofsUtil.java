package frc.robot.util.poofsUtils;

import java.util.List;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/**
 * Contains basic functions that are used often.
 */
public class PoofsUtil {
    public static final double kEpsilon = 1e-12;

    /**
     * Prevent this class from being instantiated.
     */
    private PoofsUtil() {}

    /**
     * Limits the given input to the given magnitude.
     */
    public static double limit(double v, double maxMagnitude) {
        return limit(v, -maxMagnitude, maxMagnitude);
    }

    public static double limit(double v, double min, double max) {
        return Math.min(max, Math.max(min, v));
    }

    public static boolean inRange(double v, double maxMagnitude) {
        return inRange(v, -maxMagnitude, maxMagnitude);
    }

    /**
     * Checks if the given input is within the range (min, max), both exclusive.
     */
    public static boolean inRange(double v, double min, double max) {
        return v > min && v < max;
    }

    public static double interpolate(double a, double b, double x) {
        x = limit(x, 0.0, 1.0);
        return a + (b - a) * x;
    }

    public static String joinStrings(final String delim, final List<?> strings) {
        StringBuilder sb = new StringBuilder();
        for (int i = 0; i < strings.size(); ++i) {
            sb.append(strings.get(i).toString());
            if (i < strings.size() - 1) {
                sb.append(delim);
            }
        }
        return sb.toString();
    }

    public static boolean epsilonEquals(double a, double b, double epsilon) {
        return (a - epsilon <= b) && (a + epsilon >= b);
    }

    public static boolean epsilonEquals(double a, double b) {
        return epsilonEquals(a, b, kEpsilon);
    }

    public static boolean epsilonEquals(int a, int b, int epsilon) {
        return (a - epsilon <= b) && (a + epsilon >= b);
    }

    public static boolean epsilonEquals(final Twist2d first, final Twist2d other, double epsilon) {
        return PoofsUtil.epsilonEquals(first.dx, other.dx, epsilon) &&
               PoofsUtil.epsilonEquals(first.dy, other.dy, epsilon) &&
               PoofsUtil.epsilonEquals(first.dtheta, other.dtheta, epsilon);
    }

    public static boolean allCloseTo(final List<Double> list, double value, double epsilon) {
        boolean result = true;
        for (Double value_in : list) {
            result &= epsilonEquals(value_in, value, epsilon);
        }
        return result;
    }

    public static double handleDeadband(double value, double deadband) {
        deadband = Math.abs(deadband);
        if (deadband == 1) {
            return 0;
        }
        double scaledValue = (value + (value < 0 ? deadband : -deadband)) / (1 - deadband);
        return (Math.abs(value) > Math.abs(deadband)) ? scaledValue : 0;
    }

    public static Rotation2d rotationFlip(Rotation2d rotation) {
        return Rotation2d.fromRadians(rotation.getRadians() + Math.PI);
    }

    public static Rotation2d rotationClamp(Rotation2d rotation) {
        return Rotation2d.fromRotations(MathUtil.clamp(rotation.getRotations(), 0, 1));
    }

    public static Rotation2d rotationInverse(Rotation2d rotation) {
        return Rotation2d.fromRadians(-rotation.getRadians());
    }

    public static Twist2d toTwist2d(ChassisSpeeds s) {
        return new Twist2d(s.vxMetersPerSecond, s.vyMetersPerSecond, s.omegaRadiansPerSecond);
    }
}
