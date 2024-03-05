package frc.robot.subsystems.climb;

import org.littletonrobotics.junction.AutoLog;

public interface ClimbIO {
    @AutoLog
    public static class ClimbIOInputs {
        public double leftMotorRotations = 0.0;
        public double leftMotorVelocity = 0.0;
        public double leftMotorSuppliedCurrentAmps = 0.0;
        public double leftMotorTempCelsius = 0.0;
        public boolean leftMotorReverseSoftLimit = false;
        public boolean leftMotorForwardSoftLimit = false;

        public double rightMotorRotations = 0.0;
        public double rightMotorVelocity = 0.0;
        public double rightMotorSuppliedCurrentAmps = 0.0;
        public double rightMotorTempCelsius = 0.0;
        public boolean rightMotorReverseSoftLimit = false;
        public boolean rightMotorForwardSoftLimit = false;
    }

    public default void updateInputs(ClimbIOInputs inputs) {}

    public default void updateOutputs() {}

    public default void enablePID(boolean enabled) {}

    public default void setPositionTarget(double position) {}

    public default void setThrottle(double throttle) {}
}
