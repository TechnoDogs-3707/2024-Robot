// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI.Port;

/** Add your docs here. */
public class GyroNavXIO implements GyroIO {
    private final AHRS gyro;

    public GyroNavXIO(Port spi_port) {
        gyro = new AHRS(spi_port);
    }
    
    @Override
    public void updateInputs(GyroIOInputs inputs) {
        inputs.connected = gyro.isConnected();
        inputs.calibrating = gyro.isCalibrating();

        inputs.yawAngleRotations = gyro.getYaw() / 360.0;
        inputs.pitchAngleRotations = gyro.getPitch() / 360.0;
        inputs.rollAngleRotations = gyro.getRoll() / 360.0;

        inputs.quaternionW = gyro.getQuaternionW();
        inputs.quaternionX = gyro.getQuaternionX();
        inputs.quaternionY = gyro.getQuaternionY();
        inputs.quaternionZ = gyro.getQuaternionZ();
    }
}
