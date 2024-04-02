package frc.robot.subsystems.tilt;

public final class TiltConstants {
    public static final int kMotorID = 48;
    public static final String kMotorBus = "canivore";

    public static final boolean invertMaster = false;

    public static final double kSensorToMechanismRatio = 50.67;
        
    public static final double kG = 0.35;
    public static final double kS = 2.5;
    public static final double kV = 40.0;
    public static final double kA = 20.0;
    public static final double kP = 64.0;
    public static final double kI = 0.0;
    public static final double kD = 0.0;

    public static final double kMagicVel = 0.5;
    public static final double kMagicAccel = 5;
    public static final double kMagicJerk = 10.0;

    public static final double kLiberalAllowableError = 0.01;
    public static final double kConservativeAllowableError = 0.003;

    public static final double kHomePosition = 0.0;
    public static final double kMinTargetPosition = 0.002;
    public static final double kMaxTargetPosition = 0.08;

    public static final double kAbsoluteMinPosition = 0.002;
    public static final double kAbsoluteMaxPosition = 0.8;
}