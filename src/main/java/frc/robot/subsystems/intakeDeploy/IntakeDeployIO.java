package frc.robot.subsystems.intakeDeploy;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeDeployIO {
    @AutoLog
    public static class IntakeDeployIOInputs {
        public double rotations = 0.0;
        public double velocityRotPerSec = 0.0;
        public double suppliedCurrentAmps = 0.0;
        public double tempCelsius = 0.0;
        public boolean reverseHardLimit = false;
        public boolean forwardSoftLimit = false;
    }

    public default void updateInputs(IntakeDeployIOInputs inputs) {}

    public default void updateOutputs() {}

    public default void setPositionTarget(double positionTarget) {}
    
    public default void setManualThrottle(double manualThrottle) {}

    public default void enableManualThrottle(boolean enable) {}

    public default void setTorqueLimit(double torqueLimitAmps) {}

    public default void zeroPositon() {}
}
