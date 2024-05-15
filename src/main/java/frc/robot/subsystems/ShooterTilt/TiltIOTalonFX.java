package frc.robot.subsystems.shooterTilt;

import static frc.robot.subsystems.shooterTilt.ShooterTiltConstants.*;

import java.util.ArrayList;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import frc.robot.util.LoggedTunableNumber;
import frc.robot.lib.phoenixpro.PhoenixErrorChecker;
import frc.robot.lib.phoenixpro.TalonFXConfigHelper;

public class TiltIOTalonFX implements ShooterTiltIO {
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

    private final TalonFXConfigHelper mConfigHelper;

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

        mConfig = TalonFXConfigHelper.DefaultConfigs.getBaseConfig();
        mConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        mConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        mConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = kAbsoluteMaxPosition;
        mConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        mConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = kAbsoluteMinPosition;
        // We calculate our own feedforward within the subsystem. 
        // This is not the best solution, but I don't want to recalibrate all of the shooter positions yet.
        mConfig.Slot0.kG = 0.0;
        mConfig.Slot0.kS = kS;
        mConfig.Slot0.kV = kV;
        mConfig.Slot0.kA = kA;
        mConfig.Slot0.kP = kP;
        mConfig.Slot0.kI = kI;
        mConfig.Slot0.kD = kD;

        mConfig.MotionMagic.MotionMagicCruiseVelocity = kMagicVel;
        mConfig.MotionMagic.MotionMagicAcceleration = kMagicAccel;
        mConfig.MotionMagic.MotionMagicJerk = kMagicJerk;
        
        mConfig.Feedback.SensorToMechanismRatio = kSensorToMechanismRatio;
        
        mConfigHelper = new TalonFXConfigHelper(mMotor, mConfig);
        mConfigHelper.writeConfigs();
        mConfigHelper.setSupplyCurrentLimit(20, true);
        mConfigHelper.setStatorCurrentLimit(80, true);
        
        PhoenixErrorChecker.checkErrorAndRetry(() -> mMotor.setPosition(kHomePosition));

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
    public void updateInputs(ShooterTiltIOInputs inputs) {
        mStatusSignals.forEach((s) -> s.refresh());

        inputs.tiltForwardSoftLimit = mMotorForwardSoftLimit.getValue();
        inputs.tiltRotations = mMotorPosition.getValue();
        inputs.tiltVelocityRotPerSec = mMotorSpeed.getValue();
        inputs.tiltSuppliedCurrentAmps = mMotorCurrent.getValue();
        inputs.tiltTempCelsius = mMotorTemp.getValue();

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
