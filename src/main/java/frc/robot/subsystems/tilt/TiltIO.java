package frc.robot.subsystems.tilt;

import org.littletonrobotics.junction.AutoLog;

public interface TiltIO {
    @AutoLog
    public static class TiltIOInputs {
        public double tiltRotations = 0.0;
        public double tiltVelocityRotPerSec = 0.0;
        public double tiltSuppliedCurrentAmps = 0.0;
        public double tiltTempCelsius = 0.0;
        public boolean tiltReverseHardLimit = false;
        public boolean tiltForwardSoftLimit = false;
    }

    public default void updateInputs(TiltIOInputs inputs) {}

    public default void updateOutputs() {}

    public default void setTiltTarget(double tiltTargetRotations) {}

    public default void setTiltFeedForward(double tiltFeedForwardVolts) {}
}
