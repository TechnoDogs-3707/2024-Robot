package frc.robot.subsystems.localizer;

public final class LocalizerConstants {
    public static final double ambiguityThreshold = 0.4;
    public static final double targetLogTimeSecs = 0.1;
    public static final double fieldBorderMargin = 0.5;
    public static final double zMargin = 0.75;
    public static final double xyStdDevCoefficient = 0.005;
    public static final double thetaStdDevCoefficient = 0.01;

    public static final boolean useThetaEstimate = false;
}