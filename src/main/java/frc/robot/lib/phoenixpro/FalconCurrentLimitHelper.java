// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lib.phoenixpro;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.hardware.TalonFX;

/** Add your docs here. */
public class FalconCurrentLimitHelper {
    private static final double defaultTimeoutSeconds = 0.1;

    private final TalonFX mMotor;
    private final TalonFXConfigurator mConfigurator;

    private final CurrentLimitsConfigs mNormalLimitConfig;
    private final TorqueCurrentConfigs mFOCLimitConfig;

    public FalconCurrentLimitHelper(TalonFX motor, double defaultLimitAmps) {
        mMotor = motor;
        mConfigurator = mMotor.getConfigurator();

        mNormalLimitConfig = new CurrentLimitsConfigs();
        mFOCLimitConfig = new TorqueCurrentConfigs();

        refreshConfigs(1.0);
    }

    private void refreshConfigs() {
        refreshConfigs(defaultTimeoutSeconds);
    }

    private void refreshConfigs(double timeout) {
        PhoenixProUtil.checkErrorAndRetry(() -> mConfigurator.refresh(mNormalLimitConfig, timeout));
        PhoenixProUtil.checkErrorAndRetry(() -> mConfigurator.refresh(mFOCLimitConfig, timeout));
    }

    private void applyConfigs() {
        applyConfigs(defaultTimeoutSeconds);
    }

    private void applyConfigs(double timeout) {
        PhoenixProUtil.checkErrorAndRetry(() -> mConfigurator.apply(mNormalLimitConfig, timeout));
        PhoenixProUtil.checkErrorAndRetry(() -> mConfigurator.apply(mFOCLimitConfig, timeout));
    }

    public void setCurrentLimit(double amps) {
        refreshConfigs();
        mNormalLimitConfig.StatorCurrentLimitEnable = true;
        mNormalLimitConfig.StatorCurrentLimit = amps;

        mFOCLimitConfig.PeakForwardTorqueCurrent = amps;
        mFOCLimitConfig.PeakReverseTorqueCurrent = -amps;
        applyConfigs();
    }
}
