package frc.robot.subsystems.flywheels;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.lib.phoenixpro.PhoenixErrorChecker;
import frc.robot.lib.phoenixpro.TalonFXConfigHelper;
import frc.robot.util.LoggedTunableNumber;

import static frc.robot.subsystems.flywheels.FlywheelsConstants.*;

import java.util.ArrayList;

public class FlywheelsIOTalonFX implements FlywheelsIO {
    private final TalonFX mLeftMotor;
    private final TalonFX mRightMotor;

    private final TalonFXConfiguration mRightMotorConfig;
    private final TalonFXConfiguration mLeftMotorConfig;
    private final TalonFXConfigHelper mLeftConfigHelper;
    private final TalonFXConfigHelper mRightConfigHelper;

    private final LoggedTunableNumber mTunableKS = new LoggedTunableNumber("Flywheels/kS", kS);
    private final LoggedTunableNumber mTunableKV = new LoggedTunableNumber("Flywheels/kV", kV);
    private final LoggedTunableNumber mTunableKA = new LoggedTunableNumber("Flywheels/kA", kA);
    private final LoggedTunableNumber mTunableKP = new LoggedTunableNumber("Flywheels/kP", kP);
    private final LoggedTunableNumber mTunableKI = new LoggedTunableNumber("Flywheels/kI", kI);
    private final LoggedTunableNumber mTunableKD = new LoggedTunableNumber("Flywheels/kD", kD);
    
    private final VelocityVoltage mOutputControlLeft;
    private final VelocityVoltage mOutputControlRight;
    private final NeutralOut mSpindownOutputControl;

    private final StatusSignal<Double> mLeftMotorSpeed;
    private final StatusSignal<Double> mLeftMotorCurrent;
    private final StatusSignal<Double> mLeftMotorTemp;
    private final StatusSignal<Double> mRightMotorSpeed;
    private final StatusSignal<Double> mRightMotorCurrent;
    private final StatusSignal<Double> mRightMotorTemp;

    private final ArrayList<StatusSignal<?>> mStatusSignals;

    private double mSetpointSpeedLeft = 0.0;
    private double mSetpointSpeedRight = 0.0;
    private boolean mWasBrakeMode = false;
    private boolean mBrakeMode = true;

    private boolean mSpinDownMode = false;

    public FlywheelsIOTalonFX() {
        mLeftMotor = new TalonFX(kLeftMotorID, kMotorBus);
        mRightMotor = new TalonFX(kRightMotorID, kMotorBus);

        mRightMotorConfig = TalonFXConfigHelper.DefaultConfigs.getBaseConfig();
        mRightMotorConfig.Slot0.kS = kS;
        mRightMotorConfig.Slot0.kV = kV;
        mRightMotorConfig.Slot0.kA = kA;
        mRightMotorConfig.Slot0.kP = kP;
        mRightMotorConfig.Slot0.kI = kI;
        mRightMotorConfig.Slot0.kD = kD;
        mRightMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        mLeftMotorConfig = TalonFXConfigHelper.DefaultConfigs.getBaseConfig();
        mLeftMotorConfig.Slot0.kS = kS;
        mLeftMotorConfig.Slot0.kV = kV;
        mLeftMotorConfig.Slot0.kA = kA;
        mLeftMotorConfig.Slot0.kP = kP;
        mLeftMotorConfig.Slot0.kI = kI;
        mLeftMotorConfig.Slot0.kD = kD;
        mLeftMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        mLeftConfigHelper = new TalonFXConfigHelper(mLeftMotor, mLeftMotorConfig);
        mLeftConfigHelper.writeConfigs();
        mLeftConfigHelper.setSupplyCurrentLimit(40, true);
        mLeftConfigHelper.setStatorCurrentLimit(100, true);

        mRightConfigHelper = new TalonFXConfigHelper(mRightMotor, mRightMotorConfig);
        mRightConfigHelper.writeConfigs();
        mRightConfigHelper.setSupplyCurrentLimit(40, true);
        mRightConfigHelper.setStatorCurrentLimit(100, true);

        mOutputControlLeft = new VelocityVoltage(0, 0, false, 0, 0, false, false, false);
        mOutputControlRight = new VelocityVoltage(0, 0, false, 0, 0, false, false, false);

        mSpindownOutputControl = new NeutralOut();

        mLeftMotorSpeed = mLeftMotor.getVelocity();
        mLeftMotorCurrent = mLeftMotor.getSupplyCurrent();
        mLeftMotorTemp = mLeftMotor.getDeviceTemp();
        mRightMotorSpeed = mRightMotor.getVelocity();
        mRightMotorCurrent = mRightMotor.getSupplyCurrent();
        mRightMotorTemp = mRightMotor.getDeviceTemp();

        mStatusSignals = new ArrayList<>();
        mStatusSignals.add(mLeftMotorSpeed);
        mStatusSignals.add(mLeftMotorCurrent);
        mStatusSignals.add(mLeftMotorTemp);
        mStatusSignals.add(mRightMotorSpeed);
        mStatusSignals.add(mRightMotorCurrent);
        mStatusSignals.add(mRightMotorTemp);
    }

    @Override
    public void updateInputs(FlywheelsIOInputs inputs) {
        mStatusSignals.forEach((s) -> s.refresh());

        inputs.leftMotorSpeedRPS = mLeftMotorSpeed.getValue();
        inputs.leftMotorSuppliedCurrentAmps = mLeftMotorCurrent.getValue();
        inputs.leftMotorTempCelsius = mLeftMotorCurrent.getValue();
        inputs.rightMotorSpeedRPS = mRightMotorSpeed.getValue();
        inputs.rightMotorSuppliedCurrentAmps = mRightMotorCurrent.getValue();
        inputs.rightMotorTempCelsius = mRightMotorTemp.getValue();

        mTunableKS.ifChanged(hashCode(), mLeftConfigHelper::setKS, mRightConfigHelper::setKS);
        mTunableKV.ifChanged(hashCode(), mLeftConfigHelper::setKV, mRightConfigHelper::setKV);
        mTunableKA.ifChanged(hashCode(), mLeftConfigHelper::setKA, mRightConfigHelper::setKA);
        mTunableKP.ifChanged(hashCode(), mLeftConfigHelper::setKP, mRightConfigHelper::setKP);
        mTunableKI.ifChanged(hashCode(), mLeftConfigHelper::setKI, mRightConfigHelper::setKI);
        mTunableKD.ifChanged(hashCode(), mLeftConfigHelper::setKD, mRightConfigHelper::setKD);
    }

    @Override
    public void updateOutputs() {
        if (mBrakeMode != mWasBrakeMode) {
            mLeftConfigHelper.setNeutralModeUnchecked(mBrakeMode ? NeutralModeValue.Brake : NeutralModeValue.Coast);
            mRightConfigHelper.setNeutralModeUnchecked(mBrakeMode ? NeutralModeValue.Brake : NeutralModeValue.Coast);
            mWasBrakeMode = mBrakeMode;
        }

        mOutputControlLeft.Velocity = mSetpointSpeedLeft;
        mOutputControlRight.Velocity = mSetpointSpeedRight;

        if (mSpinDownMode) {
            mLeftMotor.setControl(mSpindownOutputControl);
            mRightMotor.setControl(mSpindownOutputControl);
        } else {
            mLeftMotor.setControl(mOutputControlLeft);
            mRightMotor.setControl(mOutputControlRight);
        }
    }

    @Override
    public void setSpeedSetpointLeft(double setpointSpeedRPS) {
        mSetpointSpeedLeft = setpointSpeedRPS;
    }

    @Override
    public void setSpeedSetpointRight(double setpointSpeedRPS) {
        mSetpointSpeedRight = setpointSpeedRPS;
    }

    @Override
    public void setBrakeMode(boolean brakeMode) {
        mBrakeMode = brakeMode;
    }

    @Override
    public void setSpinDownMode(boolean spindownMode) {
        mSpinDownMode = spindownMode;
    }
}
