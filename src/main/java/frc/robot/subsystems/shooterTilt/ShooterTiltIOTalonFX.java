package frc.robot.subsystems.shooterTilt;

import java.util.ArrayList;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.lib.phoenixpro.PhoenixProUtil;
import frc.robot.lib.phoenixpro.TalonConfigHelper;

import static frc.robot.Constants.ShooterTilt.*;

public class ShooterTiltIOTalonFX implements ShooterTiltIO {
    private final TalonFX mMotor;

    private final TalonFXConfiguration mConfig;

    private final MotionMagicVoltage mControl;

    private final StatusSignal<Double> mMotorPosition;
    private final StatusSignal<Double> mMotorSpeed;
    private final StatusSignal<Double> mMotorCurrent;
    private final StatusSignal<Double> mMotorTemp;
    private final StatusSignal<Boolean> mMotorForwardSoftLimit;
    
    private final ArrayList<StatusSignal<?>> mStatusSignals;

    private double tiltTargetRotations = 0.0;
    private double tiltFeedforwardVoltage = 0.0;

    public ShooterTiltIOTalonFX() {
        mMotor = new TalonFX(kMotorID, kMotorBus);

        mConfig = TalonConfigHelper.getBaseConfig();
        mConfig.Slot0.kP = 0.0;
        mConfig.Slot0.kI = 0.0;
        mConfig.Slot0.kD = 0.0;
        mConfig.Slot0.kV = 0.0;
        mConfig.Slot0.kA = 0.0;

        mConfig.Feedback.SensorToMechanismRatio = 50.67;

        PhoenixProUtil.checkErrorAndRetry(() -> mMotor.getConfigurator().apply(mConfig));
        PhoenixProUtil.checkErrorAndRetry(() -> mMotor.setPosition(kHomePosition));
        
        mControl = new MotionMagicVoltage(0, true, 0, 0, false, false, false);

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
        inputs.tiltVelocityRotPerSec = mMotorPosition.getValue();
        inputs.tiltSuppliedCurrentAmps = mMotorCurrent.getValue();
        inputs.tiltTempCelsius = mMotorTemp.getValue();
    }

    @Override
    public void updateOutputs() {
        mControl.Position = tiltTargetRotations;
        mControl.FeedForward = tiltFeedforwardVoltage;

        mMotor.setControl(mControl);
    }
}
