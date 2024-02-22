// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lib.phoenixpro;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;

/** Add your docs here. */
public class TalonConfigHelper {
    // private static final TalonFXConfiguration kBaseConfig = new TalonFXConfiguration();

    public static final TalonFXConfiguration getBaseConfig() {
        TalonFXConfiguration kBaseConfig = new TalonFXConfiguration();

        kBaseConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        kBaseConfig.CurrentLimits.SupplyCurrentThreshold = 60;
        kBaseConfig.CurrentLimits.SupplyTimeThreshold = 0.2;
        kBaseConfig.CurrentLimits.SupplyCurrentLimit = 40;

        kBaseConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        
        kBaseConfig.TorqueCurrent.PeakForwardTorqueCurrent = 40;
        kBaseConfig.TorqueCurrent.PeakReverseTorqueCurrent = -40;

        kBaseConfig.Voltage.PeakForwardVoltage = 12;
        kBaseConfig.Voltage.PeakReverseVoltage = -12;
        
        return kBaseConfig;
    }

}
