package frc.robot.subsystems.flywheels;

import org.littletonrobotics.junction.AutoLog;

public interface FlywheelsIO {
    @AutoLog
    public static class FlywheelsIOInputs {
        public double leftMotorSpeedRPS = 0.0;
        public double leftMotorSuppliedCurrentAmps = 0.0;
        public double leftMotorTempCelsius = 0.0;

        public double rightMotorSpeedRPS = 0.0;
        public double rightMotorSuppliedCurrentAmps = 0.0;
        public double rightMotorTempCelsius = 0.0;
    }

    public default void updateInputs(FlywheelsIOInputs inputs) {}

    public default void updateOutputs() {}

    public default void setSpeedSetpointLeft(double setpointSpeedRPS) {}

    public default void setSpeedSetpointRight(double setpointSpeedRPS) {}

    public default void setBrakeMode(boolean brakeMode) {}

    public default void setSpinDownMode(boolean spindownMode) {}
}
