package frc.robot.subsystems.climb;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;

import frc.robot.lib.phoenixpro.PhoenixProUtil;
import frc.robot.lib.phoenixpro.TalonFXConfigHelper;
import frc.robot.lib.phoenixpro.TalonFXFeedbackControlHelper;
import frc.robot.util.LoggedTunableNumber;

import static frc.robot.Constants.Climb.*;

import java.util.ArrayList;

public class ClimbIOTalonFX implements ClimbIO {
    private final TalonFX mLeftMotor;
    private final TalonFX mRightMotor;

    private final TalonFXConfiguration mLeftMotorConfig;
    private final TalonFXConfiguration mRightMotorConfig;

    private final DutyCycleOut mDutyCycle;
    private final MotionMagicVoltage mPosition;

    private final StatusSignal<Double> leftMotorPosition;
    private final StatusSignal<Double> leftMotorVelocity;
    private final StatusSignal<Double> leftMotorSuppliedCurrent;
    private final StatusSignal<Double> leftMotorTemperature;
    private final StatusSignal<Boolean> leftMotorReverseSoftLimit;
    private final StatusSignal<Boolean> leftMotorForwardsSoftLimit;

    private final StatusSignal<Double> rightMotorPosition;
    private final StatusSignal<Double> rightMotorVelocity;
    private final StatusSignal<Double> rightMotorSuppliedCurrent;
    private final StatusSignal<Double> rightMotorTemperature;
    private final StatusSignal<Boolean> rightMotorReverseSoftLimit;
    private final StatusSignal<Boolean> rightMotorForwardsSoftLimit;

    private final ArrayList<StatusSignal<?>> mStatusSignals;

    private final LoggedTunableNumber mTunableKG = new LoggedTunableNumber("Climb/kG", kG);
    private final LoggedTunableNumber mTunableKS = new LoggedTunableNumber("Climb/kS", kS);
    private final LoggedTunableNumber mTunableKV = new LoggedTunableNumber("Climb/kV", kV);
    private final LoggedTunableNumber mTunableKA = new LoggedTunableNumber("Climb/kA", kA);
    private final LoggedTunableNumber mTunableKP = new LoggedTunableNumber("Climb/kP", kP);
    private final LoggedTunableNumber mTunableKI = new LoggedTunableNumber("Climb/kI", kI);
    private final LoggedTunableNumber mTunableKD = new LoggedTunableNumber("Climb/kD", kD);
    
    private final LoggedTunableNumber mTunableMagicVel = new LoggedTunableNumber("Climb/MagicVelocity", kMagicVel);
    private final LoggedTunableNumber mTunableMagicAccel = new LoggedTunableNumber("Climb/MagicAcceleration", kMagicAccel);

    private final TalonFXFeedbackControlHelper mFeedbackHelperLeft;
    private final TalonFXFeedbackControlHelper mFeedbackHelperRight;

    private boolean mEnablePID = false;

    public ClimbIOTalonFX() {
        mLeftMotor = new TalonFX(kLeftMotorID, kMotorBus);
        mRightMotor = new TalonFX(kRightMotorID, kMotorBus);

        mLeftMotorConfig = TalonFXConfigHelper.getBaseConfig();
        mLeftMotorConfig.MotorOutput.Inverted = leftMotorPolarity;
        mLeftMotorConfig.Slot0.GravityType = GravityTypeValue.Elevator_Static;
        mLeftMotorConfig.Slot0.kG = kG;
        mLeftMotorConfig.Slot0.kS = kS;
        mLeftMotorConfig.Slot0.kV = kV;
        mLeftMotorConfig.Slot0.kA = kA;
        mLeftMotorConfig.Slot0.kP = kP;
        mLeftMotorConfig.Slot0.kI = kI;
        mLeftMotorConfig.Slot0.kD = kD;
        mLeftMotorConfig.MotionMagic.MotionMagicCruiseVelocity = kMagicVel;
        mLeftMotorConfig.MotionMagic.MotionMagicAcceleration = kMagicAccel;
        mLeftMotorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        mLeftMotorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = kForwardSoftLimitValue;
        mLeftMotorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        mLeftMotorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = kReverseSoftLimitValue;

        PhoenixProUtil.checkErrorAndRetry(() -> mLeftMotor.getConfigurator().apply(mLeftMotorConfig));
        PhoenixProUtil.checkErrorAndRetry(() -> mLeftMotor.setPosition(kMotorHomePosition));

        mRightMotorConfig = TalonFXConfigHelper.getBaseConfig();
        mRightMotorConfig.MotorOutput.Inverted = rightMotorPolarity;
        mRightMotorConfig.Slot0.GravityType = GravityTypeValue.Elevator_Static;
        mRightMotorConfig.Slot0.kG = kG;
        mRightMotorConfig.Slot0.kS = kS;
        mRightMotorConfig.Slot0.kV = kV;
        mRightMotorConfig.Slot0.kA = kA;
        mRightMotorConfig.Slot0.kP = kP;
        mRightMotorConfig.Slot0.kI = kI;
        mRightMotorConfig.Slot0.kD = kD;
        mRightMotorConfig.MotionMagic.MotionMagicCruiseVelocity = kMagicVel;
        mRightMotorConfig.MotionMagic.MotionMagicAcceleration = kMagicAccel;
        mRightMotorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        mRightMotorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = kForwardSoftLimitValue;
        mRightMotorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        mRightMotorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = kReverseSoftLimitValue;

        PhoenixProUtil.checkErrorAndRetry(() -> mRightMotor.getConfigurator().apply(mRightMotorConfig));
        PhoenixProUtil.checkErrorAndRetry(() -> mRightMotor.setPosition(kMotorHomePosition));

        mDutyCycle = new DutyCycleOut(0, true, false, false, false);
        mPosition = new MotionMagicVoltage(0, true, 0, 0, false, false, false);

        leftMotorPosition = mLeftMotor.getPosition();
        leftMotorVelocity = mLeftMotor.getVelocity();
        leftMotorSuppliedCurrent = mLeftMotor.getSupplyCurrent();
        leftMotorTemperature = mLeftMotor.getDeviceTemp();
        leftMotorReverseSoftLimit = mLeftMotor.getFault_ReverseSoftLimit();
        leftMotorForwardsSoftLimit = mLeftMotor.getFault_ForwardSoftLimit();

        rightMotorPosition = mRightMotor.getPosition();
        rightMotorVelocity = mRightMotor.getVelocity();
        rightMotorSuppliedCurrent = mRightMotor.getSupplyCurrent();
        rightMotorTemperature = mRightMotor.getDeviceTemp();
        rightMotorReverseSoftLimit = mRightMotor.getFault_ReverseSoftLimit();
        rightMotorForwardsSoftLimit = mRightMotor.getFault_ForwardSoftLimit();

        mStatusSignals = new ArrayList<>();

        mStatusSignals.add(leftMotorPosition);
        mStatusSignals.add(leftMotorVelocity);
        mStatusSignals.add(leftMotorSuppliedCurrent);
        mStatusSignals.add(leftMotorTemperature);
        mStatusSignals.add(leftMotorReverseSoftLimit);
        mStatusSignals.add(leftMotorForwardsSoftLimit);

        mStatusSignals.add(rightMotorPosition);
        mStatusSignals.add(rightMotorVelocity);
        mStatusSignals.add(rightMotorSuppliedCurrent);
        mStatusSignals.add(rightMotorTemperature);
        mStatusSignals.add(rightMotorReverseSoftLimit);
        mStatusSignals.add(rightMotorForwardsSoftLimit);

        mFeedbackHelperLeft = new TalonFXFeedbackControlHelper(mLeftMotor, mLeftMotorConfig.Slot0, mLeftMotorConfig.MotionMagic);
        mFeedbackHelperRight = new TalonFXFeedbackControlHelper(mRightMotor, mRightMotorConfig.Slot0, mRightMotorConfig.MotionMagic);
    }

    @Override
    public void updateInputs(ClimbIOInputs inputs) {
        mStatusSignals.forEach((s) -> s.refresh());

        inputs.leftMotorRotations = leftMotorPosition.getValue();
        inputs.leftMotorVelocity = leftMotorVelocity.getValue();
        inputs.leftMotorSuppliedCurrentAmps = leftMotorSuppliedCurrent.getValue();
        inputs.leftMotorTempCelsius = leftMotorTemperature.getValue();
        inputs.leftMotorReverseSoftLimit = leftMotorReverseSoftLimit.getValue();
        inputs.leftMotorForwardSoftLimit = leftMotorForwardsSoftLimit.getValue();

        inputs.rightMotorRotations = rightMotorPosition.getValue();
        inputs.rightMotorVelocity = rightMotorVelocity.getValue();
        inputs.rightMotorSuppliedCurrentAmps = rightMotorSuppliedCurrent.getValue();
        inputs.rightMotorTempCelsius = rightMotorTemperature.getValue();
        inputs.rightMotorReverseSoftLimit = rightMotorReverseSoftLimit.getValue();
        inputs.rightMotorForwardSoftLimit = rightMotorForwardsSoftLimit.getValue();
        
        mTunableKG.ifChanged(hashCode(), mFeedbackHelperLeft::setKG, mFeedbackHelperRight::setKG);
        mTunableKS.ifChanged(hashCode(), mFeedbackHelperLeft::setKS, mFeedbackHelperRight::setKS);
        mTunableKV.ifChanged(hashCode(), mFeedbackHelperLeft::setKV, mFeedbackHelperRight::setKV);
        mTunableKA.ifChanged(hashCode(), mFeedbackHelperLeft::setKA, mFeedbackHelperRight::setKA);
        mTunableKP.ifChanged(hashCode(), mFeedbackHelperLeft::setKP, mFeedbackHelperRight::setKP);
        mTunableKI.ifChanged(hashCode(), mFeedbackHelperLeft::setKI, mFeedbackHelperRight::setKI);
        mTunableKD.ifChanged(hashCode(), mFeedbackHelperLeft::setKD, mFeedbackHelperRight::setKD);

        mTunableMagicVel.ifChanged(hashCode(), mFeedbackHelperLeft::setMagicVelocity, mFeedbackHelperRight::setMagicVelocity);
        mTunableMagicAccel.ifChanged(hashCode(), mFeedbackHelperLeft::setMagicAcceleration, mFeedbackHelperRight::setMagicAcceleration);
    }

    @Override
    public void updateOutputs() {
        if (mEnablePID) {
            mLeftMotor.setControl(mPosition);
            mRightMotor.setControl(mPosition);
        } else {
            mLeftMotor.setControl(mDutyCycle);
            mRightMotor.setControl(mDutyCycle);
        }
    }

    @Override
    public void enablePID(boolean enabled) {
        mEnablePID = enabled;
    }

    @Override
    public void setPositionTarget(double position) {
        mPosition.Position = position;
    }

    @Override
    public void setThrottle(double throttle) {
        mDutyCycle.Output = throttle;
    }
}
