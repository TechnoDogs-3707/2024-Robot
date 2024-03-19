// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lib.phoenixpro;

import java.util.function.UnaryOperator;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

/** Add your docs here. */
public class TalonFXConfigHelper {
    // private static final TalonFXConfiguration kBaseConfig = new TalonFXConfiguration();
    private final TalonFX mMotor;
    private TalonFXConfiguration mConfig;

    public TalonFXConfigHelper(TalonFX motor, TalonFXConfiguration config) {
        mMotor = motor;
        mConfig = config;
    }

    public void setKG(Double kG) {
        changeTalonConfig((config) -> {config.Slot0.kG = kG; return config;});
    }

    public void setKS(Double kS) {
        changeTalonConfig((config) -> {config.Slot0.kS = kS; return config;});
    }

    public void setKV(Double kV) {
        changeTalonConfig((config) -> {config.Slot0.kV = kV; return config;});
    }

    public void setKA(Double kA) {
        changeTalonConfig((config) -> {config.Slot0.kA = kA; return config;});
    }

    public void setKP(Double kP) {
        changeTalonConfig((config) -> {config.Slot0.kP = kP; return config;});
    }

    public void setKI(Double kI) {
        changeTalonConfig((config) -> {config.Slot0.kI = kI; return config;});
    }

    public void setKD(Double kD) {
        changeTalonConfig((config) -> {config.Slot0.kD = kD; return config;});
    }

    public void setMagicVelocity(Double magicVelocity) {
        changeTalonConfig((config) -> {config.MotionMagic.MotionMagicCruiseVelocity = magicVelocity; return config;});
    }

    public void setMagicAcceleration(Double magicAcceleration) {
        changeTalonConfig((config) -> {config.MotionMagic.MotionMagicAcceleration = magicAcceleration; return config;});
    }

    public void setMagicJerk(Double magicJerk) {
        changeTalonConfig((config) -> {config.MotionMagic.MotionMagicJerk = magicJerk; return config;});
    }

    public void setStatorCurrentLimit(double currentLimit, boolean enable) {
        changeTalonConfig((config) -> {
            config.CurrentLimits.StatorCurrentLimit = currentLimit;
            config.CurrentLimits.StatorCurrentLimitEnable = enable;
            return config;
        });
    }

    public void enableSoftLimits(boolean enable) {
        changeTalonConfig((config) -> {
            config.SoftwareLimitSwitch.ForwardSoftLimitEnable = enable;
            config.SoftwareLimitSwitch.ReverseSoftLimitEnable = enable;
            return config;
        });
    }

    public void setNeutralMode(NeutralModeValue mode) {
        changeTalonConfig((config) -> {config.MotorOutput.NeutralMode = mode; return config;});
    }

    public void changeTalonConfig(UnaryOperator<TalonFXConfiguration> configChanger) {
        mConfig = configChanger.apply(mConfig);
        writeConfigs();
    }

    public void writeConfigs() {
        TalonFXUtil.applyAndCheckConfiguration(mMotor, mConfig);
    }

    public synchronized void setSupplyCurrentLimit(double value, boolean enable) {
        mConfig.CurrentLimits.SupplyCurrentLimit = value;
        mConfig.CurrentLimits.SupplyCurrentLimitEnable = enable;

        TalonFXUtil.applyAndCheckConfiguration(mMotor, mConfig);
    }

    public synchronized void setSupplyCurrentLimitUnchecked(double value, boolean enable) {
        mConfig.CurrentLimits.SupplyCurrentLimit = value;
        mConfig.CurrentLimits.SupplyCurrentLimitEnable = enable;

        mMotor.getConfigurator().apply(mConfig);
    }

    public synchronized void setStatorCurrentLimitUnchecked(double value, boolean enable) {
        mConfig.CurrentLimits.StatorCurrentLimit = value;
        mConfig.CurrentLimits.StatorCurrentLimitEnable = enable;

        mMotor.getConfigurator().apply(mConfig);
    }

    public static class DefaultConfigs {
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
    
    
    
}
