// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface SwerveModuleIO {
    @AutoLog
    public static class SwerveModuleIOInputs {
        public double driveMeters = 0.0;
        public double driveVelocityMetersPerSec = 0.0;
        public double driveSuppliedCurrentAmps = 0.0;
        public double driveTempCelsius = 0.0;

        public double steerPositionRotations = 0.0;
        public double steerVelocityRotPerSec = 0.0;
        public double steerSuppliedCurrentAmps = 0.0;
        public double steerTempCelsius = 0.0;

        public double steerEncoderRaw = 0.0;
    }

    public default void updateInputs(SwerveModuleIOInputs inputs) {}

    public default void updateOutputs() {}

    public default void setDriveSpeedClosedLoop(double speedMetersPerSecond) {}
    
    public default void setDriveThrottleOpenLoop(double throttle) {}

    public default void setDriveUseOpenLoop(boolean enabledThrottle) {}

    public default void setSteerPositionTarget(double steerAngleRotations) {}

    public default void setSteerVoltageOpenLoop(double steerVoltage) {}
    
    public default void setSteerUseOpenLoop(boolean enableManualVoltage) {}

    public default void setDriveBrakeMode(boolean driveBrakeMode) {}

    public default void setSteerBrakeMode(boolean steerBrakeMode) {}

    public default void setDriveKP(double driveKP) {}

    public default void setDriveKI(double driveKI) {}

    public default void setDriveKD(double drivekD) {}

    public default void setDriveKV(double drivekV) {}

    public default void setDriveKS(double driveKS) {}

    public default void setSteerKP(double steerKP) {}

    public default void setSteerKI(double steerKI) {}

    public default void setSteerKD(double steerKD) {}

    public default void setSteerKV(double steerKV) {}

    public default void setSteerKS(double steerKS) {}

    public default void setSteerKA(double steerKA) {}

    public default void updateEncoderOffset(double zeroRotations) {}
    
    public default double getEncoderOffset() {return 0.0;};

    public default double getEncoderRawPosition() {return 0.0;}

    public default void stop() {}
}
