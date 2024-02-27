package frc.robot.subsystems.arm;

import java.util.ArrayList;
import java.util.Collection;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.ReverseLimitValue;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;
import static frc.robot.Constants.ArmSubsystem.*;

import frc.robot.lib.dashboard.LoggedTunableNumber;
import frc.robot.lib.phoenixpro.PhoenixProUtil;
import frc.robot.lib.phoenixpro.TalonFXConfigHelper;
import frc.robot.lib.phoenixpro.TalonFXFeedbackControlHelper;

// TODO: account for the virtual four-bar created by the fact that the J2 drive chain is coaxial to the J1 axis.

public class ArmIOTalonFX implements ArmIO {
    ////////// TILT MOTORS \\\\\\\\\\
    private final TalonFX mTiltMotorMaster;
    private final TalonFX mTiltMotorFollower;

    private final TalonFXConfiguration mTiltConfig;

    private final MotionMagicVoltage mTiltControlMaster;
    private final StrictFollower mTiltControlFollower;

    private final TalonFXFeedbackControlHelper mFeedbackHelperTiltMaster;
    private final TalonFXFeedbackControlHelper mFeedbackHelperTiltFollower;

    private final LoggedTunableNumber mTunableTiltKS = new LoggedTunableNumber("Arm/Tilt/kS", J1.kS);
    private final LoggedTunableNumber mTunableTiltKV = new LoggedTunableNumber("Arm/Tilt/kV", J1.kV);
    private final LoggedTunableNumber mTunableTiltKA = new LoggedTunableNumber("Arm/Tilt/kA", J1.kA);
    private final LoggedTunableNumber mTunableTiltKP = new LoggedTunableNumber("Arm/Tilt/kP", J1.kP);
    private final LoggedTunableNumber mTunableTiltKI = new LoggedTunableNumber("Arm/Tilt/kI", J1.kI);
    private final LoggedTunableNumber mTunableTiltKD = new LoggedTunableNumber("Arm/Tilt/kD", J1.kD);

    ////////// WRIST MOTORS \\\\\\\\\\
    private final TalonFX mWristMotorMaster;

    private final TalonFXConfiguration mWristConfig;

    private final MotionMagicVoltage mWristControlMaster;

    private final TalonFXFeedbackControlHelper mFeedbackHelperWrist;

    private final LoggedTunableNumber mTunableWristKS = new LoggedTunableNumber("Arm/Wrist/kS", J2.kS);
    private final LoggedTunableNumber mTunableWristKV = new LoggedTunableNumber("Arm/Wrist/kV", J2.kV);
    private final LoggedTunableNumber mTunableWristKA = new LoggedTunableNumber("Arm/Wrist/kA", J2.kA);
    private final LoggedTunableNumber mTunableWristKP = new LoggedTunableNumber("Arm/Wrist/kP", J2.kP);
    private final LoggedTunableNumber mTunableWristKI = new LoggedTunableNumber("Arm/Wrist/kI", J2.kI);
    private final LoggedTunableNumber mTunableWristKD = new LoggedTunableNumber("Arm/Wrist/kD", J2.kD);

    ///////// STATUS SIGNALS \\\\\\\\\\
    private StatusSignal<Double> tiltMasterPosition;
    private StatusSignal<Double> tiltMasterVelocity;
    private StatusSignal<Double> tiltMasterSuppliedCurrent;
    private StatusSignal<Double> tiltMasterTempCelsius;
    private StatusSignal<ReverseLimitValue> tiltMasterReverseHardLimit;
    private StatusSignal<Boolean> tiltMasterForwardSoftLimit;

    private StatusSignal<Double> tiltFollowerPosition;
    private StatusSignal<Double> tiltFollowerVelocity;
    private StatusSignal<Double> tiltFollowerSuppliedCurrent;
    private StatusSignal<Double> tiltFollowerTempCelsius;
    private StatusSignal<ReverseLimitValue> tiltFollowerReverseHardLimit;
    private StatusSignal<Boolean> tiltFollowerForwardSoftLimit;

    private StatusSignal<Double> wristMasterPosition;
    private StatusSignal<Double> wristMasterVelocity;
    private StatusSignal<Double> wristMasterSuppliedCurrent;
    private StatusSignal<Double> wristMasterTempCelsius;
    private StatusSignal<ReverseLimitValue> wristMasterReverseHardLimit;
    private StatusSignal<Boolean> wristMasterForwardSoftLimit;

    private Collection<StatusSignal<?>> m_signals = new ArrayList<StatusSignal<?>>();

    public ArmIOTalonFX() {
        ////////// TILT MOTORS \\\\\\\\\\
        mTiltMotorMaster = new TalonFX(J1.kMasterMotorID, J1.kMotorBus);
        mTiltMotorFollower = new TalonFX(J1.kFollowerMotorID, J1.kMotorBus);
        mTiltConfig = TalonFXConfigHelper.getBaseConfig();
        
        mTiltConfig.Slot0.kS = Constants.ArmSubsystem.J1.kS;
        mTiltConfig.Slot0.kV = Constants.ArmSubsystem.J1.kV;
        mTiltConfig.Slot0.kA = Constants.ArmSubsystem.J1.kA;
        mTiltConfig.Slot0.kP = Constants.ArmSubsystem.J1.kP;
        mTiltConfig.Slot0.kI = Constants.ArmSubsystem.J1.kI;
        mTiltConfig.Slot0.kD = Constants.ArmSubsystem.J1.kD;
        
        mTiltConfig.MotionMagic.MotionMagicCruiseVelocity = Constants.ArmSubsystem.J1.kMagicVel;
        mTiltConfig.MotionMagic.MotionMagicAcceleration = Constants.ArmSubsystem.J1.kMagicAccel;
        mTiltConfig.MotionMagic.MotionMagicJerk = Constants.ArmSubsystem.J1.kMagicJerk;

        mTiltConfig.Feedback.SensorToMechanismRatio = 110.73;

        mTiltControlMaster = new MotionMagicVoltage(0, true, 0, 0, false, false, false);
        mTiltControlFollower = new StrictFollower(mTiltMotorMaster.getDeviceID());

        mFeedbackHelperTiltMaster = new TalonFXFeedbackControlHelper(mTiltMotorMaster, mTiltConfig.Slot0, mTiltConfig.MotionMagic);
        mFeedbackHelperTiltFollower = new TalonFXFeedbackControlHelper(mTiltMotorFollower, mTiltConfig.Slot0, mTiltConfig.MotionMagic);

        ////////// WRIST MOTOR \\\\\\\\\\
        mWristMotorMaster = new TalonFX(J2.kMasterMotorID, J2.kMotorBus);
        
        mWristConfig = TalonFXConfigHelper.getBaseConfig();
        
        mWristConfig.Slot0.kS = Constants.ArmSubsystem.J2.kS;
        mWristConfig.Slot0.kV = Constants.ArmSubsystem.J2.kV;
        mWristConfig.Slot0.kA = Constants.ArmSubsystem.J2.kA;
        mWristConfig.Slot0.kP = Constants.ArmSubsystem.J2.kP;
        mWristConfig.Slot0.kI = Constants.ArmSubsystem.J2.kI;
        mWristConfig.Slot0.kD = Constants.ArmSubsystem.J2.kD;

        mWristConfig.MotionMagic.MotionMagicCruiseVelocity = Constants.ArmSubsystem.J2.kMagicVel;
        mWristConfig.MotionMagic.MotionMagicAcceleration = Constants.ArmSubsystem.J2.kMagicAccel;
        mWristConfig.MotionMagic.MotionMagicJerk = Constants.ArmSubsystem.J2.kMagicJerk;

        mWristConfig.Feedback.SensorToMechanismRatio = 48.0;

        mWristControlMaster = new MotionMagicVoltage(0, true, 0, 0, false, false, false);

        mFeedbackHelperWrist = new TalonFXFeedbackControlHelper(mWristMotorMaster, mWristConfig.Slot0, mWristConfig.MotionMagic);

        ////////// INTAKE MOTORS \\\\\\\\\\
        

        ////////// ALL MOTORS \\\\\\\\\\
        configMotors();
        refreshFollowers();

        ////////// STATUS SIGNALS \\\\\\\\\\
        tiltMasterPosition = mTiltMotorMaster.getPosition();
        tiltMasterVelocity = mTiltMotorMaster.getVelocity();
        tiltMasterSuppliedCurrent = mTiltMotorMaster.getSupplyCurrent();
        tiltMasterTempCelsius = mTiltMotorMaster.getDeviceTemp();
        tiltMasterForwardSoftLimit = mTiltMotorMaster.getFault_ForwardSoftLimit();
        tiltMasterReverseHardLimit = mTiltMotorMaster.getReverseLimit();

        tiltFollowerPosition = mTiltMotorFollower.getPosition();
        tiltFollowerVelocity = mTiltMotorFollower.getVelocity();
        tiltFollowerSuppliedCurrent = mTiltMotorFollower.getSupplyCurrent();
        tiltFollowerTempCelsius = mTiltMotorFollower.getDeviceTemp();
        tiltFollowerForwardSoftLimit = mTiltMotorFollower.getFault_ForwardSoftLimit();
        tiltFollowerReverseHardLimit = mTiltMotorFollower.getReverseLimit();

        wristMasterPosition = mWristMotorMaster.getPosition();
        wristMasterVelocity = mWristMotorMaster.getVelocity();
        wristMasterSuppliedCurrent = mWristMotorMaster.getSupplyCurrent();
        wristMasterTempCelsius = mWristMotorMaster.getDeviceTemp();
        wristMasterForwardSoftLimit = mWristMotorMaster.getFault_ForwardSoftLimit();
        wristMasterReverseHardLimit = mWristMotorMaster.getReverseLimit();
        

        m_signals.add(tiltMasterPosition);
        m_signals.add(tiltMasterVelocity);
        m_signals.add(tiltMasterSuppliedCurrent);
        m_signals.add(tiltMasterTempCelsius);
        m_signals.add(tiltMasterForwardSoftLimit);
        m_signals.add(tiltMasterReverseHardLimit);

        m_signals.add(tiltFollowerPosition);
        m_signals.add(tiltFollowerVelocity);
        m_signals.add(tiltFollowerSuppliedCurrent);
        m_signals.add(tiltFollowerTempCelsius);
        m_signals.add(tiltFollowerForwardSoftLimit);
        m_signals.add(tiltFollowerReverseHardLimit);

        m_signals.add(wristMasterPosition);
        m_signals.add(wristMasterVelocity);
        m_signals.add(wristMasterSuppliedCurrent);
        m_signals.add(wristMasterTempCelsius);
        m_signals.add(wristMasterForwardSoftLimit);
        m_signals.add(wristMasterReverseHardLimit);
    }

    private void configMotors() {
        PhoenixProUtil.checkErrorAndRetry(() -> mTiltMotorMaster.getConfigurator().apply(mTiltConfig));
        PhoenixProUtil.checkErrorAndRetry(() -> mTiltMotorFollower.getConfigurator().apply(mTiltConfig));

        PhoenixProUtil.checkErrorAndRetry(() -> mTiltMotorMaster.setPosition(J1.kHomePosition));
        PhoenixProUtil.checkErrorAndRetry(() -> mTiltMotorFollower.setPosition(J1.kHomePosition));

        PhoenixProUtil.checkErrorAndRetry(() -> mWristMotorMaster.getConfigurator().apply(mWristConfig));
        PhoenixProUtil.checkErrorAndRetry(() -> mWristMotorMaster.setPosition(J2.kHomePosition));
    }

    @Override
    public void updateInputs(ArmIOInputs inputs) {
        m_signals.forEach((s) -> s.refresh());

        inputs.tiltRotations = tiltMasterPosition.getValue();
        inputs.tiltVelocityRotPerSec = tiltMasterVelocity.getValue();
        inputs.tiltSuppliedCurrentAmps = 0;
        inputs.tiltSuppliedCurrentAmps += tiltMasterSuppliedCurrent.getValue();
        inputs.tiltSuppliedCurrentAmps += tiltFollowerSuppliedCurrent.getValue();
        inputs.tiltHottestTempCelsius = Math.max(tiltMasterTempCelsius.getValue(), tiltFollowerTempCelsius.getValue());
        inputs.tiltReverseHardLimit = tiltMasterReverseHardLimit.getValue() == ReverseLimitValue.ClosedToGround;
        inputs.tiltForwardSoftLimit = tiltMasterForwardSoftLimit.getValue();

        inputs.wristRotations = wristMasterPosition.getValue();
        inputs.wristVelocityRotPerSec = wristMasterVelocity.getValue();
        inputs.wristSuppliedCurrentAmps = 0;
        inputs.wristSuppliedCurrentAmps += wristMasterSuppliedCurrent.getValue();
        inputs.wristHottestTempCelsius = wristMasterTempCelsius.getValue();
        inputs.wristReverseHardLimit = wristMasterReverseHardLimit.getValue() == ReverseLimitValue.ClosedToGround;
        inputs.wristForwardSoftLimit = wristMasterForwardSoftLimit.getValue();

        mTunableTiltKS.ifChanged(hashCode(), mFeedbackHelperTiltMaster::setKS, mFeedbackHelperTiltFollower::setKS);
        mTunableTiltKV.ifChanged(hashCode(), mFeedbackHelperTiltMaster::setKV, mFeedbackHelperTiltFollower::setKV);
        mTunableTiltKA.ifChanged(hashCode(), mFeedbackHelperTiltMaster::setKA, mFeedbackHelperTiltFollower::setKA);
        mTunableTiltKP.ifChanged(hashCode(), mFeedbackHelperTiltMaster::setKP, mFeedbackHelperTiltFollower::setKP);
        mTunableTiltKI.ifChanged(hashCode(), mFeedbackHelperTiltMaster::setKI, mFeedbackHelperTiltFollower::setKI);
        mTunableTiltKD.ifChanged(hashCode(), mFeedbackHelperTiltMaster::setKD, mFeedbackHelperTiltFollower::setKD);

        mTunableWristKS.ifChanged(hashCode(), mFeedbackHelperWrist::setKS);
        mTunableWristKV.ifChanged(hashCode(), mFeedbackHelperWrist::setKV);
        mTunableWristKA.ifChanged(hashCode(), mFeedbackHelperWrist::setKA);
        mTunableWristKP.ifChanged(hashCode(), mFeedbackHelperWrist::setKP);
        mTunableWristKI.ifChanged(hashCode(), mFeedbackHelperWrist::setKI);
        mTunableWristKD.ifChanged(hashCode(), mFeedbackHelperWrist::setKD);
    }

    @Override
    public void setTiltTarget(double rotations) {
        mTiltControlMaster.Position = rotations;
    }

    @Override
    public void setTiltFeedForward(double amps) {
        mTiltControlMaster.FeedForward = amps;
    }

    @Override
    public void setWristTarget(double rotations) {
        mWristControlMaster.Position = rotations;
    }

    @Override
    public void setWristFeedForward(double amps) {
        mWristControlMaster.FeedForward = amps;
    }

    

    @Override
    public void updateOutputs() {
        mTiltMotorMaster.setControl(mTiltControlMaster);
        mWristMotorMaster.setControl(mWristControlMaster);
    }

    @Override
    public void refreshFollowers() {
        mTiltMotorFollower.setControl(mTiltControlFollower);
    }
}
