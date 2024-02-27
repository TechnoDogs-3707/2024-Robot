package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    @AutoLog
    public static class IntakeIOInputs {
        public double intakeVelocityRotPerSec = 0.0;
        public double intakeSuppliedCurrentAmps = 0.0;
        public double intakeHottestTempCelsius = 0.0;
        public boolean intakeBeamBreakTriggered = false;
    }

    public default void updateInputs(IntakeIOInputs inputs) {}

    public default void setIntakeThrottle(double throttle) {}

    public default void updateOutputs() {}
}
