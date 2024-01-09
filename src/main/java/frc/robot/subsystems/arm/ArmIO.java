package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
    @AutoLog
    public static class ArmIOInputs {
        public double tiltRotations = 0.0;
        public double tiltVelocityRotPerSec = 0.0;
        public double tiltSuppliedCurrentAmps = 0.0;
        public double tiltHottestTempCelsius = 0.0;
        public boolean tiltReverseHardLimit = false;
        public boolean tiltForwardSoftLimit = false;

        public double extendMeters = 0.0;
        public double extendVelocityMetersPerSec = 0.0;
        public double extendSuppliedCurrentAmps = 0.0;
        public double extendHottestTempCelsius = 0.0;
        public boolean extendReverseHardLimit = false;
        public boolean extendForwardSoftLimit = false;

        public double wristRotations = 0.0;
        public double wristVelocityRotPerSec = 0.0;
        public double wristSuppliedCurrentAmps = 0.0;
        public double wristTempCelsius = 0.0;
        public boolean wristReverseHardLimit = false;
        public boolean wristForwardSoftLimit = false;
    }

    public default void updateInputs(ArmIOInputs inputs) {}

    public default void updateOutputs() {}

    public default void refreshFollowers() {}

    public default void setTiltTarget(double rotations) {}

    public default void setExtendTarget(double meters) {}

    public default void setWristTarget(double rotations) {}

    public default void setTiltFeedForward(double amps) {}

    public default void setExtendFeedForward(double amps) {}

    public default void setWristFeedForward(double amps) {}
}
