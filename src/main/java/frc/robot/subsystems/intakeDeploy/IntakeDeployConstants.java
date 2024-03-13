package frc.robot.subsystems.intakeDeploy;

public final class IntakeDeployConstants {
    public static final int kMasterMotorID = 32;
    public static final String kMotorBus = "canivore"; // Both motors must be on same bus to use follower mode

    public static final boolean invertMaster = false;
    
    public static final double kG = 0.2;

    public static final double kS = 0.0;
    public static final double kV = 5.0;
    public static final double kA = 0.0;
    public static final double kP = 14.0;
    public static final double kI = 0.0;
    public static final double kD = 0.0;

    public static final double kMagicVel = 1.0;
    public static final double kMagicAccel = 10.0;
    public static final double kMagicJerk = 0.0;

    public static final double kLiberalAllowableError = 0.06;
    public static final double kConservativeAllowableError = 0.04;

    public static final double kHomePosition = 0.42;
    public static final double kMinTargetPosition = 0.0;
    public static final double kMaxTargetPosition = 0.0;

    public static final double kAbsoluteMaxPosition = 0.0;
    public static final double kAbsoluteMinPosition = 0.0;
}