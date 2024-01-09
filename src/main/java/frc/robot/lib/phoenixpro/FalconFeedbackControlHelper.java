// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lib.phoenixpro;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;

/** Add your docs here. */
public class FalconFeedbackControlHelper {
    private static final double kTimeout = 0.1;

    private final TalonFXConfigurator mConfigurator;

    private final Slot0Configs mPIDConfigs;
    private final MotionMagicConfigs mMagicConfigs;

    public FalconFeedbackControlHelper(TalonFX motor, Slot0Configs defaultPID, MotionMagicConfigs defaultMagic) {
        mConfigurator = motor.getConfigurator();

        if (defaultPID == null) {
            defaultPID = new Slot0Configs();
        }

        if (defaultMagic == null) {
            defaultMagic = new MotionMagicConfigs();
        }

        mPIDConfigs = defaultPID;
        mMagicConfigs = defaultMagic;

        ensureDefaultConfigsApplied(mPIDConfigs, mMagicConfigs);
        refreshConfigs();
    }

    public FalconFeedbackControlHelper(TalonFX motor, Slot0Configs defaultPID) {
        this(motor, defaultPID, null);
    }

    public FalconFeedbackControlHelper(TalonFX motor) {
        mConfigurator = motor.getConfigurator();
        
        mPIDConfigs = new Slot0Configs();
        mMagicConfigs = new MotionMagicConfigs();

        refreshConfigs();
    }

    private void ensureDefaultConfigsApplied(Slot0Configs defaultPID, MotionMagicConfigs defaultMagic) {
        ensureDefaultConfigsApplied(defaultPID, defaultMagic, kTimeout);
    }

    private void ensureDefaultConfigsApplied(Slot0Configs defaultPID, MotionMagicConfigs defaultMagic, double timeout) {
        PhoenixProUtil.checkErrorAndRetry(() -> mConfigurator.apply(defaultPID, timeout));
        PhoenixProUtil.checkErrorAndRetry(() -> mConfigurator.apply(defaultMagic, timeout));
        refreshConfigs(timeout);
    }

    private void refreshConfigs() {
        refreshConfigs(kTimeout);
    }

    private void refreshConfigs(double timeout) {
        PhoenixProUtil.checkErrorAndRetry(() -> mConfigurator.refresh(mPIDConfigs, timeout));
        PhoenixProUtil.checkErrorAndRetry(() -> mConfigurator.refresh(mMagicConfigs, timeout));
    }

    private void applyConfigs() {
        applyConfigs(kTimeout);
    }

    private void applyConfigs(double timeout) {
        PhoenixProUtil.checkErrorAndRetry(() -> mConfigurator.apply(mPIDConfigs, timeout));
        PhoenixProUtil.checkErrorAndRetry(() -> mConfigurator.apply(mMagicConfigs, timeout));
    }

    public void setKP(double kP) {
        refreshConfigs();
        mPIDConfigs.kP = kP;
        applyConfigs();
    }

    public void setKI(double kI) {
        refreshConfigs();
        mPIDConfigs.kI = kI;
        applyConfigs();
    }

    public void setKD(double kD) {
        refreshConfigs();
        mPIDConfigs.kD = kD;
        applyConfigs();
    }

    public void setKV(double kV) {
        refreshConfigs();
        mPIDConfigs.kV= kV;
        applyConfigs();
    }

    public void setKS(double kS) {
        refreshConfigs();
        mPIDConfigs.kS = kS;
        applyConfigs();
    }

    public void setMagicVelocity(double magicVelocity) {
        refreshConfigs();
        mMagicConfigs.MotionMagicCruiseVelocity = magicVelocity;
        applyConfigs();
    }

    public void setMagicAcceleration(double magicAcceleration) {
        refreshConfigs();
        mMagicConfigs.MotionMagicAcceleration = magicAcceleration;
        applyConfigs();
    }

    public void setMagicJerk(double magicJerk) {
        refreshConfigs();
        mMagicConfigs.MotionMagicJerk = magicJerk;
        applyConfigs();
    }

    public double getKP() {
        refreshConfigs();
        return mPIDConfigs.kP;
    }

    public double getKI() {
        refreshConfigs();
        return mPIDConfigs.kI;
    }

    public double getKD() {
        refreshConfigs();
        return mPIDConfigs.kD;
    }

    public double getKV() {
        refreshConfigs();
        return mPIDConfigs.kV;
    }

    public double getMagicVelocity() {
        refreshConfigs();
        return mMagicConfigs.MotionMagicCruiseVelocity;
    }

    public double getMagicAcceleration() {
        refreshConfigs();
        return mMagicConfigs.MotionMagicAcceleration;
    }

    public double getMagicJerk() {
        refreshConfigs();
        return mMagicConfigs.MotionMagicJerk;
    }
}
