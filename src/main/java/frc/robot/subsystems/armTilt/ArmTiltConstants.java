package frc.robot.subsystems.armTilt;

import com.ctre.phoenix6.signals.InvertedValue;

public final class ArmTiltConstants {
    public static final int kMotorID = 30;
    public static final String kMotorBus = "canivore"; // Both motors must be on same bus to use follower mode

    public static final InvertedValue mInverted = InvertedValue.CounterClockwise_Positive;
    
    public static final double kG = 0.352;

    public static final double kS = 0.0;
    public static final double kV = 0.0;
    public static final double kA = 0.0;
    public static final double kP = 0.0;
    public static final double kI = 0.0;
    public static final double kD = 0.0;

    public static final double kMagicVel = 0.05;
    public static final double kMagicAccel = 0.5;
    public static final double kMagicJerk = 0.0;

    public static final double kLiberalAllowableError = 0.06;
    public static final double kConservativeAllowableError = 0.04;

    public static final double kHomePosition = 0.0;
    public static final double kMinTargetPosition = 0.0;
    public static final double kMaxTargetPosition = 0.0;

    public static final double kHomingThrottle = -0.1;
    public static final double kHomingVelocityThreshold = 0.01;
    public static final double kHomingMinTime = 0.25;

    public static final double kAbsoluteMaxPosition = 0.25;
    public static final double kAbsoluteMinPosition = 0.0;
}