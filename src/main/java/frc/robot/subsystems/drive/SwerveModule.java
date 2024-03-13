// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardBoolean;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

import frc.robot.Constants;
import frc.robot.lib.dashboard.LoggedTunableNumber;

/** Add your docs here. */
public class SwerveModule {
    private final SwerveModuleIO io;
    private final SwerveModuleIOInputsAutoLogged inputs = new SwerveModuleIOInputsAutoLogged();
    private final int index;

    private static final LoggedTunableNumber drivekS = new LoggedTunableNumber("Drive/Module/Drive/KS", Constants.DriveSubsystem.kDrivePIDConfig.kS);
    private static final LoggedTunableNumber driveKV = new LoggedTunableNumber("Drive/Module/Drive/KV", Constants.DriveSubsystem.kDrivePIDConfig.kV);
    private static final LoggedTunableNumber driveKP = new LoggedTunableNumber("Drive/Module/Drive/KP", Constants.DriveSubsystem.kDrivePIDConfig.kP);
    private static final LoggedTunableNumber driveKI = new LoggedTunableNumber("Drive/Module/Drive/KI", Constants.DriveSubsystem.kDrivePIDConfig.kI);
    private static final LoggedTunableNumber driveKD = new LoggedTunableNumber("Drive/Module/Drive/KD", Constants.DriveSubsystem.kDrivePIDConfig.kD);
    
    private static final LoggedTunableNumber steerKS = new LoggedTunableNumber("Drive/Module/Steer/KS", Constants.DriveSubsystem.kSteerPIDConfig.kS);
    private static final LoggedTunableNumber steerKV = new LoggedTunableNumber("Drive/Module/Steer/KV", Constants.DriveSubsystem.kSteerPIDConfig.kV);
    private static final LoggedTunableNumber steerKP = new LoggedTunableNumber("Drive/Module/Steer/KP", Constants.DriveSubsystem.kSteerPIDConfig.kP);
    private static final LoggedTunableNumber steerKI = new LoggedTunableNumber("Drive/Module/Steer/KI", Constants.DriveSubsystem.kSteerPIDConfig.kI);
    private static final LoggedTunableNumber steerKD = new LoggedTunableNumber("Drive/Module/Steer/KD", Constants.DriveSubsystem.kSteerPIDConfig.kD);
    
    private static final LoggedDashboardBoolean steerVoltageOverride = new LoggedDashboardBoolean("Drive/Module/Steer/OverrideVoltage", false);
    private boolean overrideSteerVoltage = false;
    private static final LoggedTunableNumber steerMotorVoltage = new LoggedTunableNumber("Drive/Module/Steer/ManualVoltage", 0);

    public static final SendableChooser<Boolean> steerNeutralMode = new SendableChooser<Boolean>();
    private boolean steerIsNeutral = false;

    public boolean getSteerNeutralMode() {
        return steerIsNeutral;
    }

    public SwerveModule(SwerveModuleIO io, int index) {
        this.io = io;
        this.index = index;

        steerNeutralMode.setDefaultOption("Brake", false);
        steerNeutralMode.addOption("Coast", true);
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Drive/Module" + Integer.toString(index), inputs);

        if (driveKP.hasChanged(hashCode())) {
            io.setDriveKP(driveKP.get());
        }

        if (driveKI.hasChanged(hashCode())) {
            io.setDriveKI(driveKI.get());
        }

        if (driveKD.hasChanged(hashCode())) {
            io.setDriveKD(driveKD.get());
        }

        if (driveKV.hasChanged(hashCode())) {
            io.setDriveKV(driveKV.get());
        }

        if (drivekS.hasChanged(hashCode())) {
            io.setDriveKS(drivekS.get());
        }

        if (steerKP.hasChanged(hashCode())) {
            io.setSteerKP(steerKP.get());
        }

        if (steerKI.hasChanged(hashCode())) {
            io.setSteerKI(steerKI.get());
        }

        if (steerKD.hasChanged(hashCode())) {
            io.setSteerKD(steerKD.get());
        }

        if (steerKV.hasChanged(hashCode())) {
            io.setSteerKV(steerKV.get());
        }

        if (steerKS.hasChanged(hashCode())) {
            io.setSteerKS(steerKS.get());
        }

        if (steerNeutralMode.getSelected() != steerIsNeutral) {
            io.setSteerBrakeMode(!steerNeutralMode.getSelected());
            steerIsNeutral = steerNeutralMode.getSelected();
        }

        if (steerVoltageOverride.get() != overrideSteerVoltage) {
            overrideSteerVoltage = steerVoltageOverride.get();
        }

        io.updateOutputs();
    }

    public double getVelocity() {
        return inputs.driveVelocityMetersPerSec;
    }

    public double getDistance() {
        return inputs.driveMeters;
    }

    public Rotation2d getAngle() {
        return Rotation2d.fromRotations(inputs.steerPositionRotations);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getVelocity(), getAngle());
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getDistance(), getAngle());
    }

    public SwerveModuleState setStateClosedLoop(SwerveModuleState state) {
        io.setDriveSpeedClosedLoop(state.speedMetersPerSecond);
        setSteerTarget(state.angle);
        return state;
    }

    public SwerveModuleState setStateOpenLoop(SwerveModuleState state) {
        io.setDriveThrottleOpenLoop(state.speedMetersPerSecond / Constants.kMaxVelocityMetersPerSecond);
        setSteerTarget(state.angle);
        return state;
    }

    private void setSteerTarget(Rotation2d angle) {
        if (overrideSteerVoltage) {
            io.setSteerVoltageOpenLoop(steerMotorVoltage.get());
        } else {
            io.setSteerPositionTarget(angle.getRotations());
        }
    }

    public void stop() {
        io.stop();
    }

    public void setDriveBrakeMode(boolean brake) {
        io.setDriveBrakeMode(brake);
    }

    public void setSteerBrakeMode(boolean brake) {
        io.setSteerBrakeMode(brake);
    }

    public double getTotalCurrent() {
        return inputs.driveSuppliedCurrentAmps + inputs.steerSuppliedCurrentAmps;
    }

    public void updateEncoderOffset(double zeroRotations) {
        io.updateEncoderOffset(zeroRotations);
    }

    public double getEncoderOffset() {
        return io.getEncoderOffset();
    }

    public double getEncoderRawPosition() {
        return io.getEncoderRawPosition();
    }

    public void setCurrentLimit(double amps) {
        io.setCurrentLimit(amps);
    }
}
