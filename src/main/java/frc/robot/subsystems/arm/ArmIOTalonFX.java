package frc.robot.subsystems.arm;

import java.util.ArrayList;
import java.util.Collection;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ReverseLimitValue;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;
import static frc.robot.Constants.ArmSubsystem.*;
import frc.robot.lib.phoenixpro.PhoenixProUtil;
import frc.robot.lib.phoenixpro.TalonFXConfigHelper;

// TODO: account for the virtual four-bar created by the fact that the J2 drive chain is coaxial to the J1 axis.

public class ArmIOTalonFX implements ArmIO {
    ////////// TILT MOTORS \\\\\\\\\\
    private final TalonFX mTiltMotorMaster;
    private final TalonFX mTiltMotorFollower;

    private final TalonFXConfiguration mTiltConfig;

    private final MotionMagicTorqueCurrentFOC mTiltControlMaster;
    private final StrictFollower mTiltControlFollower;

    ////////// WRIST MOTORS \\\\\\\\\\
    private final TalonFX mWristMotorMaster;

    private final TalonFXConfiguration mWristConfig;

    // private final MotionMagicTorqueCurrentFOC mWristControlMaster;
    private final PositionVoltage mWristControlMaster;

    ////////// INTAKE MOTORS \\\\\\\\\\
    private final TalonFX mIntakeMotorMaster;

    private final TalonFXConfiguration mIntakeConfig;

    private final DutyCycleOut mIntakeControlMaster;

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

    private StatusSignal<Double> intakeMasterVelocity;
    private StatusSignal<Double> intakeMasterSuppliedCurrent;
    private StatusSignal<Double> intakeMasterTempCelsius;

    private Collection<StatusSignal<?>> m_signals = new ArrayList<StatusSignal<?>>();

    private final DigitalInput mIntakeSensor;

    public ArmIOTalonFX() {
        ////////// TILT MOTORS \\\\\\\\\\
        // TODO: get motor IDs from constants
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

        mTiltControlMaster = new MotionMagicTorqueCurrentFOC(0, 0, 0, false, false, false);
        mTiltControlFollower = new StrictFollower(mTiltMotorMaster.getDeviceID());

        ////////// WRIST MOTOR \\\\\\\\\\
        mWristMotorMaster = new TalonFX(J2.kMasterMotorID, J2.kMotorBus);
        
        mWristConfig = TalonFXConfigHelper.getBaseConfig();

        mWristConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        mWristConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        mWristConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        mWristConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 0.5;
        mWristConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        mWristConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0.0;
        
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

        // mWristControlMaster = new MotionMagicTorqueCurrentFOC(0, 0, 0, false, false, false);
        mWristControlMaster = new PositionVoltage(0, 0, true, 0, 0, false, false, false);

        ////////// INTAKE MOTORS \\\\\\\\\\
        mIntakeMotorMaster = new TalonFX(Intake.kMasterMotorID, Intake.kMotorBus);

        mIntakeConfig = TalonFXConfigHelper.getBaseConfig(); //TODO: set up intake config
        mIntakeConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        // mIntakeConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        mIntakeControlMaster = new DutyCycleOut(0, false, false, false, false);

        ////////// ALL MOTORS \\\\\\\\\\
        configMotors();
        refreshFollowers();

        mIntakeSensor = new DigitalInput(0);

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

        intakeMasterVelocity = mIntakeMotorMaster.getVelocity();
        intakeMasterSuppliedCurrent = mIntakeMotorMaster.getSupplyCurrent();
        intakeMasterTempCelsius = mIntakeMotorMaster.getDeviceTemp();
        

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

        m_signals.add(intakeMasterVelocity);
        m_signals.add(intakeMasterSuppliedCurrent);
        m_signals.add(intakeMasterTempCelsius);
    }

    private void configMotors() {
        PhoenixProUtil.checkErrorAndRetry(() -> mTiltMotorMaster.getConfigurator().apply(mTiltConfig));
        PhoenixProUtil.checkErrorAndRetry(() -> mTiltMotorFollower.getConfigurator().apply(mTiltConfig));

        PhoenixProUtil.checkErrorAndRetry(() -> mTiltMotorMaster.setPosition(J1.kHomePosition));
        PhoenixProUtil.checkErrorAndRetry(() -> mTiltMotorFollower.setPosition(J1.kHomePosition));

        PhoenixProUtil.checkErrorAndRetry(() -> mWristMotorMaster.getConfigurator().apply(mWristConfig));

        PhoenixProUtil.checkErrorAndRetry(() -> mWristMotorMaster.setPosition(J2.kHomePosition));

        PhoenixProUtil.checkErrorAndRetry(() -> mIntakeMotorMaster.getConfigurator().apply(mIntakeConfig));
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

        inputs.intakeVelocityRotPerSec = intakeMasterVelocity.getValue();
        inputs.intakeSuppliedCurrentAmps = intakeMasterSuppliedCurrent.getValue();
        inputs.intakeHottestTempCelsius = intakeMasterTempCelsius.getValue();

        inputs.intakeBeamBreakTriggered = !mIntakeSensor.get();
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
    public void setIntakeThrottle(double throttle) {
        mIntakeControlMaster.Output = throttle;
    }

    @Override
    public void updateOutputs() {
        mTiltMotorMaster.setControl(mTiltControlMaster);
        mWristMotorMaster.setControl(mWristControlMaster);
        mIntakeMotorMaster.setControl(mIntakeControlMaster);
    }

    @Override
    public void refreshFollowers() {
        mTiltMotorFollower.setControl(mTiltControlFollower);
    }
}
