// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface GyroIO {
    @AutoLog
    public static class GyroIOInputs {
        public double yawAngleRotations = 0.0;
        public double pitchAngleRotations = 0.0;
        public double rollAngleRotations =0.0;

        public double yawVelocityRotationsPerSecond = 0.0;
        public double pitchVelocityRotationsPerSecond = 0.0;
        public double rollVelocityRotationsPerSecond = 0.0;

        public boolean connected = false;
        public boolean calibrating = false;

        public double quaternionW = 0.0;
        public double quaternionX = 0.0;
        public double quaternionY = 0.0;
        public double quaternionZ = 0.0;
    }

    public default void updateInputs(GyroIOInputs inputs) {}

    public default void calibrate() {}
}
