package frc.robot.subsystems.armTilt;

import org.littletonrobotics.junction.AutoLog;

public interface ArmTiltIO {
    @AutoLog
    public static class ArmTiltIOInputs {
        public double rotations = 0.0;
        public double velocityRotPerSec = 0.0;
        public double suppliedCurrentAmps = 0.0;
        public double tempCelsius = 0.0;
        public boolean reverseHardLimit = false;
        public boolean forwardSoftLimit = false;
    }

    public default void updateInputs(ArmTiltIOInputs inputs) {}

    public default void updateOutputs() {}

    public default void setPositionTarget(double positionTarget) {}
    
    public default void setManualThrottle(double manualThrottle) {}

    public default void enableManualThrottle(boolean enable) {}

    public default void setTorqueLimit(double torqueLimitAmps) {}

    public default void zeroPositon() {}
}
