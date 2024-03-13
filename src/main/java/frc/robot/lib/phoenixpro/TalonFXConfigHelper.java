// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lib.phoenixpro;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;

/** Add your docs here. */
public class TalonFXConfigHelper {
    // private static final TalonFXConfiguration kBaseConfig = new TalonFXConfiguration();

    public static final TalonFXConfiguration getBaseConfig() {
        TalonFXConfiguration baseConfig = new TalonFXConfiguration();

        baseConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        baseConfig.CurrentLimits.SupplyCurrentThreshold = 60;
        baseConfig.CurrentLimits.SupplyTimeThreshold = 0.2;
        baseConfig.CurrentLimits.SupplyCurrentLimit = 40;

        baseConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        
        baseConfig.TorqueCurrent.PeakForwardTorqueCurrent = 40;
        baseConfig.TorqueCurrent.PeakReverseTorqueCurrent = -40;

        baseConfig.Voltage.PeakForwardVoltage = 12;
        baseConfig.Voltage.PeakReverseVoltage = -12;
        
        return baseConfig;
    }

    public static final CurrentLimitsConfigs getDefaultCurrentLimits() {
        CurrentLimitsConfigs config = new CurrentLimitsConfigs();

        config.StatorCurrentLimitEnable = true;
        config.StatorCurrentLimit = 80;

        config.SupplyCurrentLimitEnable = true;
        config.SupplyCurrentThreshold = 0.0;
        config.SupplyTimeThreshold = 0.0;
        config.SupplyCurrentLimit = 40.0;

        return config;
    }

    public static final CurrentLimitsConfigs getDriveBaseCurrentLimits() {
        CurrentLimitsConfigs config = new CurrentLimitsConfigs();

        config.StatorCurrentLimitEnable = true;
        config.StatorCurrentLimit = 80;

        config.SupplyCurrentLimitEnable = true;
        config.SupplyCurrentThreshold = 0.0;
        config.SupplyTimeThreshold = 0.0;
        config.SupplyCurrentLimit = 40.0;

        return config;
    }

    public static final CurrentLimitsConfigs get20ACurrentLimits() {
        CurrentLimitsConfigs config = new CurrentLimitsConfigs();

        config.StatorCurrentLimitEnable = true;
        config.StatorCurrentLimit = 40;

        config.SupplyCurrentLimitEnable = true;
        config.SupplyCurrentThreshold = 0.0;
        config.SupplyTimeThreshold = 0.0;
        config.SupplyCurrentLimit = 20.0;

        return config;
    }

}
