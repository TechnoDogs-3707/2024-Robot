package frc.robot.subsystems.shooterTilt;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterTiltIO {
    @AutoLog
    public static class ShooterTiltIOInputs {
        public double tiltRotations = 0.0;
        public double tiltVelocityRotPerSec = 0.0;
        public double tiltSuppliedCurrentAmps = 0.0;
        public double tiltTempCelsius = 0.0;
        public boolean tiltReverseHardLimit = false;
        public boolean tiltForwardSoftLimit = false;
    }

    public default void updateInputs(ShooterTiltIOInputs inputs) {}

    public default void updateOutputs() {}

    public default void setTiltTarget(double tiltTargetRotations) {}

    public default void setTiltFeedForward(double tiltFeedForwardVolts) {}
}
