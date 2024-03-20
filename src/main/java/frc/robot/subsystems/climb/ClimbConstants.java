package frc.robot.subsystems.climb;

import com.ctre.phoenix6.signals.InvertedValue;

public final class ClimbConstants {
    public static final int kLeftMotorID = 50;
    public static final int kRightMotorID = 51;
    public static final String kMotorBus = "canivore";

    public static final InvertedValue leftMotorPolarity = InvertedValue.Clockwise_Positive;
    public static final InvertedValue rightMotorPolarity = InvertedValue.CounterClockwise_Positive;

    public static final double kG = 0.04;
    public static final double kS = 0.0;
    public static final double kV = 0.125;
    public static final double kA = 0.0;
    public static final double kP = 1.0;
    public static final double kI = 0.0;
    public static final double kD = 0.0;

    public static final double kMagicVel = 100.0;
    public static final double kMagicAccel = 400.0;
    
    public static final double kPIDAllowableError = 1.0;

    public static final double kMotorHomePosition = -2.0;

    public static final double kReverseSoftLimitValue = 1.0;
    public static final double kForwardSoftLimitValue = 60.0;

    public static final double kClimbingThrottle = 0.0;

    public static final double kFullExtensionPosition = 59.0;
    public static final double kFullRetractionPosition = 1.0;
}