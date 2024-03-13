package frc.robot.subsystems.tilt;

import static frc.robot.subsystems.tilt.TiltConstants.*;

import java.util.ArrayList;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import frc.robot.lib.dashboard.LoggedTunableNumber;
import frc.robot.lib.phoenixpro.TalonFXFeedbackControlHelper;
import frc.robot.lib.phoenixpro.PhoenixProUtil;
import frc.robot.lib.phoenixpro.TalonFXConfigHelper;

public class TiltIOTalonFX implements TiltIO {
    private final TalonFX mMotor;

    private final TalonFXConfiguration mConfig;

    private final PositionVoltage mControl;

    private final StatusSignal<Double> mMotorPosition;
    private final StatusSignal<Double> mMotorSpeed;
    private final StatusSignal<Double> mMotorCurrent;
    private final StatusSignal<Double> mMotorTemp;
    private final StatusSignal<Boolean> mMotorForwardSoftLimit;
    
    private final ArrayList<StatusSignal<?>> mStatusSignals;

    private double tiltTargetRotations = 0.0;
    private double tiltFeedforwardVoltage = 0.0;

    private final TalonFXFeedbackControlHelper mFeedbackHelper;

    private final LoggedTunableNumber mTunableKS = new LoggedTunableNumber("Tilt/kS", kS);
    private final LoggedTunableNumber mTunableKV = new LoggedTunableNumber("Tilt/kV", kV);
    private final LoggedTunableNumber mTunableKA = new LoggedTunableNumber("Tilt/kA", kA);
    private final LoggedTunableNumber mTunableKP = new LoggedTunableNumber("Tilt/kP", kP);
    private final LoggedTunableNumber mTunableKI = new LoggedTunableNumber("Tilt/kI", kI);
    private final LoggedTunableNumber mTunableKD = new LoggedTunableNumber("Tilt/kD", kD);
    
    private final LoggedTunableNumber mTunableMagicVel = new LoggedTunableNumber("Tilt/MagicVelocity", kMagicVel);
    private final LoggedTunableNumber mTunableMagicAccel = new LoggedTunableNumber("Tilt/MagicAcceleration", kMagicAccel);
    private final LoggedTunableNumber mTunableMagicJerk = new LoggedTunableNumber("Tilt/MagicVelocity", kMagicJerk);
    
    public TiltIOTalonFX() {
        mMotor = new TalonFX(kMotorID, kMotorBus);

        mConfig = TalonFXConfigHelper.getBaseConfig();
        mConfig.CurrentLimits = TalonFXConfigHelper.get20ACurrentLimits();
        mConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        mConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        mConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 0.1;
        mConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        mConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0.0;
        mConfig.Slot0.kG = 0.0; // we calculate our own feedforward within the subsystem
        mConfig.Slot0.kS = kS;
        mConfig.Slot0.kV = kV;
        mConfig.Slot0.kA = kA;
        mConfig.Slot0.kP = kP;
        mConfig.Slot0.kI = kI;
        mConfig.Slot0.kD = kD;

        mConfig.MotionMagic.MotionMagicCruiseVelocity = kMagicVel;
        mConfig.MotionMagic.MotionMagicAcceleration = kMagicAccel;
        mConfig.MotionMagic.MotionMagicJerk = kMagicJerk;
        
        mConfig.Feedback.SensorToMechanismRatio = 50.67;
        
        PhoenixProUtil.checkErrorAndRetry(() -> mMotor.getConfigurator().apply(mConfig));
        PhoenixProUtil.checkErrorAndRetry(() -> mMotor.setPosition(kHomePosition));
        
        mFeedbackHelper = new TalonFXFeedbackControlHelper(mMotor, mConfig.Slot0, mConfig.MotionMagic);

        mControl = new PositionVoltage(0, 0, true, 0, 0, false, false, false);

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
    public void updateInputs(TiltIOInputs inputs) {
        mStatusSignals.forEach((s) -> s.refresh());

        inputs.tiltForwardSoftLimit = mMotorForwardSoftLimit.getValue();
        inputs.tiltRotations = mMotorPosition.getValue();
        inputs.tiltVelocityRotPerSec = mMotorSpeed.getValue();
        inputs.tiltSuppliedCurrentAmps = mMotorCurrent.getValue();
        inputs.tiltTempCelsius = mMotorTemp.getValue();

        mTunableKS.ifChanged(hashCode(), mFeedbackHelper::setKS);
        mTunableKV.ifChanged(hashCode(), mFeedbackHelper::setKV);
        mTunableKA.ifChanged(hashCode(), mFeedbackHelper::setKA);
        mTunableKP.ifChanged(hashCode(), mFeedbackHelper::setKP);
        mTunableKI.ifChanged(hashCode(), mFeedbackHelper::setKI);
        mTunableKD.ifChanged(hashCode(), mFeedbackHelper::setKD);

        mTunableMagicVel.ifChanged(hashCode(), mFeedbackHelper::setMagicVelocity);
        mTunableMagicAccel.ifChanged(hashCode(), mFeedbackHelper::setMagicAcceleration);
        mTunableMagicJerk.ifChanged(hashCode(), mFeedbackHelper::setMagicJerk);
    }

    @Override
    public void setTiltTarget(double tiltTargetRotations) {
        this.tiltTargetRotations = tiltTargetRotations;
    }

    @Override
    public void setTiltFeedForward(double tiltFeedForwardVolts) {
        this.tiltFeedforwardVoltage = tiltFeedForwardVolts;
    }

    @Override
    public void updateOutputs() {
        mControl.Position = tiltTargetRotations;
        mControl.FeedForward = tiltFeedforwardVoltage;

        mMotor.setControl(mControl);
    }
}
