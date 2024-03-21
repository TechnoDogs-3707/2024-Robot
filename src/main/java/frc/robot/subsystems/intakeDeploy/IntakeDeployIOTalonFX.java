package frc.robot.subsystems.intakeDeploy;

import java.util.ArrayList;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.util.LoggedTunableNumber;
import frc.robot.lib.phoenixpro.PhoenixErrorChecker;
import frc.robot.lib.phoenixpro.TalonFXConfigHelper;

import static frc.robot.subsystems.intakeDeploy.IntakeDeployConstants.*;

public class IntakeDeployIOTalonFX implements IntakeDeployIO {
    private final TalonFX mMotor;

    private final TalonFXConfiguration mConfig;

    private final MotionMagicVoltage mPIDControl;
    private final DutyCycleOut mManualControl;

    private boolean mManualOverride = false;

    private final StatusSignal<Double> mMotorPosition;
    private final StatusSignal<Double> mMotorSpeed;
    private final StatusSignal<Double> mMotorCurrent;
    private final StatusSignal<Double> mMotorTemp;
    private final StatusSignal<Boolean> mMotorForwardSoftLimit;

    private final ArrayList<StatusSignal<?>> mStatusSignals;

    private double mIntakeTargetRotations = kHomePosition;
    private double mIntakeManualThrottle = 0;

    private final TalonFXConfigHelper mConfigHelper;
    
    private final LoggedTunableNumber mTunableKG = new LoggedTunableNumber("IntakeDeploy/kG", kG);
    private final LoggedTunableNumber mTunableKS = new LoggedTunableNumber("IntakeDeploy/kS", kS);
    private final LoggedTunableNumber mTunableKV = new LoggedTunableNumber("IntakeDeploy/kV", kV);
    private final LoggedTunableNumber mTunableKA = new LoggedTunableNumber("IntakeDeploy/kA", kA);
    private final LoggedTunableNumber mTunableKP = new LoggedTunableNumber("IntakeDeploy/kP", kP);
    private final LoggedTunableNumber mTunableKI = new LoggedTunableNumber("IntakeDeploy/kI", kI);
    private final LoggedTunableNumber mTunableKD = new LoggedTunableNumber("IntakeDeploy/kD", kD);
    
    private final LoggedTunableNumber mTunableMagicVel = new LoggedTunableNumber("IntakeDeploy/MagicVelocity", kMagicVel);
    private final LoggedTunableNumber mTunableMagicAccel = new LoggedTunableNumber("IntakeDeploy/MagicAcceleration", kMagicAccel);
    private final LoggedTunableNumber mTunableMagicJerk = new LoggedTunableNumber("IntakeDeploy/MagicJerk", kMagicJerk);
    
    public IntakeDeployIOTalonFX() {
        mMotor = new TalonFX(kMotorID, kMotorBus);

        mConfig = TalonFXConfigHelper.DefaultConfigs.getBaseConfig();
        mConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        mConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        mConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        mConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = kAbsoluteMaxPosition;
        mConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        mConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = kAbsoluteMinPosition;
        mConfig.Slot0.kG = kG;
        mConfig.Slot0.kS = kS;
        mConfig.Slot0.kV = kV;
        mConfig.Slot0.kA = kA;
        mConfig.Slot0.kP = kP;
        mConfig.Slot0.kI = kI;
        mConfig.Slot0.kD = kD;

        mConfig.MotionMagic.MotionMagicCruiseVelocity = kMagicVel;
        mConfig.MotionMagic.MotionMagicAcceleration = kMagicAccel;
        mConfig.MotionMagic.MotionMagicJerk = kMagicJerk;
        
        mConfig.Feedback.SensorToMechanismRatio = 48.0;

        mConfigHelper = new TalonFXConfigHelper(mMotor, mConfig);
        mConfigHelper.writeConfigs();
        mConfigHelper.setSupplyCurrentLimit(40, true);
        mConfigHelper.setStatorCurrentLimit(20, true);
        
        PhoenixErrorChecker.checkErrorAndRetry(() -> mMotor.setPosition(kHomePosition));

        mPIDControl = new MotionMagicVoltage(0, true, 0, 0, false, false, false);
        mManualControl = new DutyCycleOut(0, true, false, false, false);

        mMotorPosition = mMotor.getPosition();
        mMotorSpeed = mMotor.getVelocity();
        mMotorCurrent = mMotor.getSupplyCurrent();
        mMotorTemp = mMotor.getDeviceTemp();
        mMotorForwardSoftLimit = mMotor.getFault_ForwardSoftLimit();

        mStatusSignals = new ArrayList<>();

        mStatusSignals.add(mMotorPosition);
        mStatusSignals.add(mMotorSpeed);
        mStatusSignals.add(mMotorCurrent);
        mStatusSignals.add(mMotorTemp);
        mStatusSignals.add(mMotorForwardSoftLimit);
    }

    @Override
    public void updateInputs(IntakeDeployIOInputs inputs) {
        mStatusSignals.forEach((s) -> s.refresh());

        inputs.rotations = mMotorPosition.getValue();
        inputs.velocityRotPerSec = mMotorSpeed.getValue();
        inputs.suppliedCurrentAmps = mMotorCurrent.getValue();
        inputs.tempCelsius = mMotorTemp.getValue();
        inputs.forwardSoftLimit = mMotorForwardSoftLimit.getValue();
        
        mTunableKG.ifChanged(hashCode(), mConfigHelper::setKG);
        mTunableKS.ifChanged(hashCode(), mConfigHelper::setKS);
        mTunableKV.ifChanged(hashCode(), mConfigHelper::setKV);
        mTunableKA.ifChanged(hashCode(), mConfigHelper::setKA);
        mTunableKP.ifChanged(hashCode(), mConfigHelper::setKP);
        mTunableKI.ifChanged(hashCode(), mConfigHelper::setKI);
        mTunableKD.ifChanged(hashCode(), mConfigHelper::setKD);

        mTunableMagicVel.ifChanged(hashCode(), mConfigHelper::setMagicVelocity);
        mTunableMagicAccel.ifChanged(hashCode(), mConfigHelper::setMagicAcceleration);
        mTunableMagicJerk.ifChanged(hashCode(), mConfigHelper::setMagicJerk);
    }

    @Override
    public void updateOutputs() {
        mManualControl.Output = mIntakeManualThrottle;
        mPIDControl.Position = mIntakeTargetRotations;

        if (mManualOverride) {
            mMotor.setControl(mManualControl);
        } else {
            mMotor.setControl(mPIDControl);
        }
    }

    @Override
    public void setPositionTarget(double positionTarget) {
        mIntakeTargetRotations = positionTarget;
    }

    @Override
    public void setManualThrottle(double manualThrottle) {
        mIntakeManualThrottle = manualThrottle;
    }

    @Override
    public void enableManualThrottle(boolean enable) {
        mManualOverride = enable;
    }

    @Override
    public void zeroPositon() {
        PhoenixErrorChecker.checkErrorAndRetry(() -> mMotor.setPosition(kHomePosition));
    }

    @Override
    public void setTorqueLimit(double torqueLimitAmps) {
        mConfigHelper.setStatorCurrentLimitUnchecked(torqueLimitAmps, true);
    }
}
