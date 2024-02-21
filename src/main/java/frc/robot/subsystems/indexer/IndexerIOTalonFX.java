// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.indexer;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.lib.phoenixpro.PhoenixProUtil;
import frc.robot.lib.phoenixpro.TalonConfigHelper;

import static frc.robot.Constants.Indexer.*;

/** Add your docs here. */
public class IndexerIOTalonFX implements IndexerIO {
    private final TalonFX mMotor;
    private final TalonFXConfiguration mConfig;

    private final DigitalInput mFirstBanner;
    private final DigitalInput mSecondBanner;

    private final DutyCycleOut mOutputControl;

    private final StatusSignal<Double> indexerSpeed;
    private final StatusSignal<Double> indexerCurrent;
    private final StatusSignal<Double> indexerTemp;

    public IndexerIOTalonFX() {
        mMotor = new TalonFX(kMotorID, kMotorBus);
        mConfig = TalonConfigHelper.getBaseConfig();

        PhoenixProUtil.checkErrorAndRetry(() -> mMotor.getConfigurator().apply(mConfig));

        mOutputControl = new DutyCycleOut(0, false, false, false, false);

        mFirstBanner = new DigitalInput(1);
        mSecondBanner = new DigitalInput(2);

        indexerSpeed = mMotor.getVelocity();
        indexerCurrent = mMotor.getSupplyCurrent();
        indexerTemp = mMotor.getDeviceTemp();
    }

    @Override
    public void updateInputs(IndexerIOInputs inputs) {
        indexerSpeed.refresh();
        indexerCurrent.refresh();
        indexerTemp.refresh();

        inputs.motorSpeedRPS = indexerSpeed.getValue();
        inputs.motorSuppliedCurrentAmps = indexerCurrent.getValue();
        inputs.motorTempCelsius = indexerTemp.getValue();

        inputs.firstBannerTriggered = !mFirstBanner.get();
        inputs.secondBannerTriggered = !mSecondBanner.get();
    }

    @Override
    public void updateOutputs() {
        mMotor.setControl(mOutputControl);
    }

    @Override
    public void setMotorThrottle(double throttle) {
        mOutputControl.Output = throttle;
    }
}
