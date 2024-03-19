// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lib.phoenixpro;

import java.util.function.UnaryOperator;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

/** Add your docs here. */
public class TalonFXLiveConfigHelper{
    /**
     * A helper function that allows a one-function change of any parameters on a TalonFX Motor Controller.
     * @param talon The TalonFX to configure.
     * @param modifier A function that is provided with the current motor config, and is expected to return
     * the updated config back, where it will be applied to the motor.
     */
    public static void editConfig(TalonFX talon, UnaryOperator<TalonFXConfiguration> modifier) {
        final TalonFXConfiguration config = new TalonFXConfiguration();
        PhoenixProUtil.checkErrorAndRetry(() -> talon.getConfigurator().refresh(config, 0.1));
        TalonFXConfiguration newConfig = modifier.apply(config);
        PhoenixProUtil.checkErrorAndRetry(() -> talon.getConfigurator().apply(newConfig, 0.1));
    }
}
