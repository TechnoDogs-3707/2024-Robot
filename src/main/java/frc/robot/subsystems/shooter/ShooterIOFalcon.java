package frc.robot.subsystems.shooter;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.lib.phoenixpro.PhoenixProUtil;
import frc.robot.lib.phoenixpro.TalonConfigHelper;
import frc.robot.lib.phoenixpro.TalonFXLiveConfigHelper;

import static frc.robot.Constants.Shooter.*;

import java.util.ArrayList;

public class ShooterIOFalcon implements ShooterIO {
    private final TalonFX mLeftMotor;
    private final TalonFX mRightMotor;

    private final TalonFXConfiguration mMotorConfig;
    
    private final VelocityVoltage mOutputControlLeft;
    private final VelocityVoltage mOutputControlRight;

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

    public ShooterIOFalcon() {
        mLeftMotor = new TalonFX(kLeftMotorID, kMotorBus);
        mRightMotor = new TalonFX(kRightMotorID, kMotorBus);

        mMotorConfig = TalonConfigHelper.getBaseConfig();
        PhoenixProUtil.checkErrorAndRetry(() -> mLeftMotor.getConfigurator().apply(mMotorConfig));
        PhoenixProUtil.checkErrorAndRetry(() -> mRightMotor.getConfigurator().apply(mMotorConfig));
        
        mOutputControlLeft = new VelocityVoltage(0, 0, false, 0, 0, false, false, false);
        mOutputControlRight = new VelocityVoltage(0, 0, false, 0, 0, false, false, false);

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
    public void updateInputs(ShooterIOInputs inputs) {
        StatusSignal.refreshAll((StatusSignal<?>[])mStatusSignals.toArray());

        inputs.leftMotorSpeedRPS = mLeftMotorSpeed.getValue();
        inputs.leftMotorSuppliedCurrentAmps = mLeftMotorCurrent.getValue();
        inputs.leftMotorTempCelsius = mLeftMotorCurrent.getValue();
        inputs.rightMotorSpeedRPS = mRightMotorSpeed.getValue();
        inputs.rightMotorSuppliedCurrentAmps = mRightMotorCurrent.getValue();
        inputs.rightMotorTempCelsius = mRightMotorTemp.getValue();
    }

    @Override
    public void updateOutputs() {
        if (mBrakeMode != mWasBrakeMode) {
            TalonFXLiveConfigHelper.editConfig(mLeftMotor, (c) -> {
                c.MotorOutput.NeutralMode = mBrakeMode ? NeutralModeValue.Brake : NeutralModeValue.Coast;
                return c;
            });
            TalonFXLiveConfigHelper.editConfig(mRightMotor, (c) -> {
                c.MotorOutput.NeutralMode = mBrakeMode ? NeutralModeValue.Brake : NeutralModeValue.Coast;
                return c;
            });
            mWasBrakeMode = mBrakeMode;
        }

        mOutputControlLeft.Velocity = mSetpointSpeedLeft;
        mOutputControlRight.Velocity = mSetpointSpeedRight;

        mLeftMotor.setControl(mOutputControlLeft);
        mRightMotor.setControl(mOutputControlRight);
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
}
