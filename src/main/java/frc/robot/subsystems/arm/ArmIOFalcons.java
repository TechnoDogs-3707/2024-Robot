package frc.robot.subsystems.arm;

import java.util.ArrayList;
import java.util.Collection;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ReverseLimitValue;

import frc.robot.Constants;
import frc.robot.lib.phoenixpro.PhoenixProUtil;
import frc.robot.lib.phoenixpro.TalonConfigHelper;

public class ArmIOFalcons implements ArmIO {
    ////////// TILT MOTORS \\\\\\\\\\
    private final TalonFX mTiltMotorMaster;
    private final TalonFX mTiltMotorFollower;

    private final TalonFXConfiguration mTiltConfig;

    private final MotionMagicTorqueCurrentFOC mTiltControlMaster;
    private final StrictFollower mTiltControlFollower;

    ////////// EXTEND MOTORS \\\\\\\\\\
    private final TalonFX mExtendMotorMaster;
    private final TalonFX mExtendMotorFollower;

    private final TalonFXConfiguration mExtendConfig;

    private final MotionMagicTorqueCurrentFOC mExtendControlMaster;
    private final StrictFollower mExtendControlFollower;

    ////////// WRIST MOTOR \\\\\\\\\\
    private final TalonFX mWristMotor;

    private final TalonFXConfiguration mWristConfig;

    private final MotionMagicTorqueCurrentFOC mWristControl;

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

    private StatusSignal<Double> extendMasterPosition;
    private StatusSignal<Double> extendMasterVelocity;
    private StatusSignal<Double> extendMasterSuppliedCurrent;
    private StatusSignal<Double> extendMasterTempCelsius;
    private StatusSignal<ReverseLimitValue> extendMasterReverseHardLimit;
    private StatusSignal<Boolean> extendMasterForwardSoftLimit;

    private StatusSignal<Double> extendFollowerPosition;
    private StatusSignal<Double> extendFollowerVelocity;
    private StatusSignal<Double> extendFollowerSuppliedCurrent;
    private StatusSignal<Double> extendFollowerTempCelsius;
    private StatusSignal<ReverseLimitValue> extendFollowerReverseHardLimit;
    private StatusSignal<Boolean> extendFollowerForwardSoftLimit;

    private StatusSignal<Double> wristPosition;
    private StatusSignal<Double> wristVelocity;
    private StatusSignal<Double> wristSuppliedCurrent;
    private StatusSignal<Double> wristTempCelsius;
    private StatusSignal<ReverseLimitValue> wristReverseHardLimit;
    private StatusSignal<Boolean> wristForwardSoftLimit;

    private Collection<StatusSignal<?>> m_signals = new ArrayList<StatusSignal<?>>();

    public ArmIOFalcons() {
        ////////// TILT MOTORS \\\\\\\\\\
        mTiltMotorMaster = new TalonFX(30, "canivore");
        mTiltMotorFollower = new TalonFX(31, "canivore");
        mTiltConfig = TalonConfigHelper.getBaseConfig();
        
        mTiltConfig.Slot0.kP = Constants.ArmSubsystem.Tilt.kP;
        mTiltConfig.Slot0.kI = Constants.ArmSubsystem.Tilt.kI;
        mTiltConfig.Slot0.kD = Constants.ArmSubsystem.Tilt.kD;
        mTiltConfig.Slot0.kV = Constants.ArmSubsystem.Tilt.kV;

        mTiltConfig.MotionMagic.MotionMagicCruiseVelocity = Constants.ArmSubsystem.Tilt.kMagicVel;
        mTiltConfig.MotionMagic.MotionMagicAcceleration = Constants.ArmSubsystem.Tilt.kMagicAccel;
        mTiltConfig.MotionMagic.MotionMagicJerk = Constants.ArmSubsystem.Tilt.kMagicJerk;

        

        mTiltControlMaster = new MotionMagicTorqueCurrentFOC(0, 0, 0, false, false, false);
        mTiltControlFollower = new StrictFollower(mTiltMotorMaster.getDeviceID());

        ////////// EXTEND MOTORS \\\\\\\\\\
        mExtendMotorMaster = new TalonFX(32, "canivore");
        mExtendMotorFollower = new TalonFX(33, "canivore");
        
        mExtendConfig = TalonConfigHelper.getBaseConfig();
        
        mExtendConfig.Slot0.kP = Constants.ArmSubsystem.Extend.kP;
        mExtendConfig.Slot0.kI = Constants.ArmSubsystem.Extend.kI;
        mExtendConfig.Slot0.kD = Constants.ArmSubsystem.Extend.kD;
        mExtendConfig.Slot0.kV = Constants.ArmSubsystem.Extend.kV;

        mExtendConfig.MotionMagic.MotionMagicCruiseVelocity = Constants.ArmSubsystem.Extend.kMagicVel;
        mExtendConfig.MotionMagic.MotionMagicAcceleration = Constants.ArmSubsystem.Extend.kMagicAccel;
        mExtendConfig.MotionMagic.MotionMagicJerk = Constants.ArmSubsystem.Extend.kMagicJerk;

        mExtendControlMaster = new MotionMagicTorqueCurrentFOC(0, 0, 0, false, false, false);

        mExtendControlFollower = new StrictFollower(mExtendMotorMaster.getDeviceID());

        ////////// WRIST MOTOR \\\\\\\\\\
        mWristMotor = new TalonFX(34, "canivore");
        
        mWristConfig = TalonConfigHelper.getBaseConfig();
        
        mWristConfig.Slot0.kP = Constants.ArmSubsystem.Wrist.kP;
        mWristConfig.Slot0.kI = Constants.ArmSubsystem.Wrist.kI;
        mWristConfig.Slot0.kD = Constants.ArmSubsystem.Wrist.kD;
        mWristConfig.Slot0.kV = Constants.ArmSubsystem.Wrist.kV;

        mWristConfig.MotionMagic.MotionMagicCruiseVelocity = Constants.ArmSubsystem.Wrist.kMagicVel;
        mWristConfig.MotionMagic.MotionMagicAcceleration = Constants.ArmSubsystem.Wrist.kMagicAccel;
        mWristConfig.MotionMagic.MotionMagicJerk = Constants.ArmSubsystem.Wrist.kMagicJerk;

        mWristControl = new MotionMagicTorqueCurrentFOC(0, 0, 0, false, false, false);

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

        extendMasterPosition = mExtendMotorMaster.getPosition();
        extendMasterVelocity = mExtendMotorMaster.getVelocity();
        extendMasterSuppliedCurrent = mExtendMotorMaster.getSupplyCurrent();
        extendMasterTempCelsius = mExtendMotorMaster.getDeviceTemp();
        extendMasterForwardSoftLimit = mExtendMotorMaster.getFault_ForwardSoftLimit();
        extendMasterReverseHardLimit = mExtendMotorMaster.getReverseLimit();

        extendFollowerPosition = mExtendMotorFollower.getPosition();
        extendFollowerVelocity = mExtendMotorFollower.getVelocity();
        extendFollowerSuppliedCurrent = mExtendMotorFollower.getSupplyCurrent();
        extendFollowerTempCelsius = mExtendMotorFollower.getDeviceTemp();
        extendFollowerForwardSoftLimit = mExtendMotorFollower.getFault_ForwardSoftLimit();
        extendFollowerReverseHardLimit = mExtendMotorFollower.getReverseLimit();

        wristPosition = mWristMotor.getPosition();
        wristVelocity = mWristMotor.getVelocity();
        wristSuppliedCurrent = mWristMotor.getSupplyCurrent();
        wristTempCelsius = mWristMotor.getDeviceTemp();
        wristForwardSoftLimit = mWristMotor.getFault_ForwardSoftLimit();
        wristReverseHardLimit = mWristMotor.getReverseLimit();

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

        m_signals.add(extendMasterPosition);
        m_signals.add(extendMasterVelocity);
        m_signals.add(extendMasterSuppliedCurrent);
        m_signals.add(extendMasterTempCelsius);
        m_signals.add(extendMasterForwardSoftLimit);
        m_signals.add(extendMasterReverseHardLimit);

        m_signals.add(extendFollowerPosition);
        m_signals.add(extendFollowerVelocity);
        m_signals.add(extendFollowerSuppliedCurrent);
        m_signals.add(extendFollowerTempCelsius);
        m_signals.add(extendFollowerForwardSoftLimit);
        m_signals.add(extendFollowerReverseHardLimit);

        m_signals.add(wristPosition);
        m_signals.add(wristVelocity);
        m_signals.add(wristSuppliedCurrent);
        m_signals.add(wristTempCelsius);
        m_signals.add(wristForwardSoftLimit);
        m_signals.add(wristReverseHardLimit);
    }

    private void configMotors() {
        PhoenixProUtil.checkErrorAndRetry(() -> mTiltMotorMaster.getConfigurator().apply(mTiltConfig));
        PhoenixProUtil.checkErrorAndRetry(() -> mTiltMotorFollower.getConfigurator().apply(mTiltConfig));

        PhoenixProUtil.checkErrorAndRetry(() -> mExtendMotorMaster.getConfigurator().apply(mExtendConfig));
        PhoenixProUtil.checkErrorAndRetry(() -> mExtendMotorFollower.getConfigurator().apply(mExtendConfig));

        PhoenixProUtil.checkErrorAndRetry(() -> mWristMotor.getConfigurator().apply(mWristConfig));
    }

    @Override
    public void updateInputs(ArmIOInputs inputs) {
        m_signals.forEach((s) -> s.refresh());

        inputs.tiltRotations = tiltMasterPosition.getValue();
        inputs.tiltVelocityRotPerSec = tiltMasterVelocity.getValue();
        inputs.tiltSuppliedCurrentAmps += tiltMasterSuppliedCurrent.getValue();
        inputs.tiltSuppliedCurrentAmps += tiltFollowerSuppliedCurrent.getValue();
        inputs.tiltHottestTempCelsius = Math.max(tiltMasterTempCelsius.getValue(), tiltFollowerTempCelsius.getValue());
        inputs.tiltForwardSoftLimit = tiltMasterForwardSoftLimit.getValue();
        inputs.tiltReverseHardLimit = tiltMasterReverseHardLimit.getValue() == ReverseLimitValue.ClosedToGround;

        inputs.extendMeters = extendMasterPosition.getValue();
        inputs.extendVelocityMetersPerSec = extendMasterVelocity.getValue();
        inputs.extendSuppliedCurrentAmps += extendMasterSuppliedCurrent.getValue();
        inputs.extendSuppliedCurrentAmps += extendFollowerSuppliedCurrent.getValue();
        inputs.extendHottestTempCelsius = Math.max(extendMasterTempCelsius.getValue(), extendFollowerTempCelsius.getValue());
        inputs.extendForwardSoftLimit = extendMasterForwardSoftLimit.getValue();
        inputs.extendReverseHardLimit = extendMasterReverseHardLimit.getValue() == ReverseLimitValue.ClosedToGround;

        inputs.wristRotations = wristPosition.getValue();
        inputs.wristVelocityRotPerSec = wristVelocity.getValue();
        inputs.wristSuppliedCurrentAmps = wristSuppliedCurrent.getValue();
        inputs.wristTempCelsius = wristTempCelsius.getValue();
        inputs.wristReverseHardLimit = wristReverseHardLimit.getValue() == ReverseLimitValue.ClosedToGround;
        inputs.wristForwardSoftLimit = wristForwardSoftLimit.getValue();
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
    public void setExtendTarget(double meters) {
        mExtendControlMaster.Position = meters;
    }

    @Override
    public void setExtendFeedForward(double amps) {
        mExtendControlMaster.FeedForward = amps;
    }

    @Override
    public void setWristTarget(double rotations) {
        mWristControl.Position = rotations;
    }

    @Override
    public void setWristFeedForward(double amps) {
        mWristControl.FeedForward = amps;
    }

    @Override
    public void updateOutputs() {
        mTiltMotorMaster.setControl(mTiltControlMaster);
        mExtendMotorMaster.setControl(mExtendControlMaster);
        mWristMotor.setControl(mWristControl);
    }

    @Override
    public void refreshFollowers() {
        mTiltMotorFollower.setControl(mTiltControlFollower);
        mExtendMotorFollower.setControl(mExtendControlFollower);
    }
}
