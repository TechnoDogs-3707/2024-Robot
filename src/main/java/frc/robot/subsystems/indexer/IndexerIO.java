package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {
    @AutoLog
    public static class IndexerIOInputs {
        public double motorSpeedRPS = 0.0;
        public double motorSuppliedCurrentAmps = 0.0;
        public double motorTempCelsius = 0.0;

        public boolean firstBannerTriggered = false;
        public boolean secondBannerTriggered = false;
    }

    public default void updateInputs(IndexerIOInputs inputs) {}

    public default void updateOutputs() {}

    public default void setMotorThrottle(double throttle) {}
}
