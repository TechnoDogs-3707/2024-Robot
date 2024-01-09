package frc.robot.subsystems.arm;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ForwardLimitValue;
import com.ctre.phoenix6.signals.ReverseLimitValue;

import frc.robot.lib.phoenixpro.PhoenixProUtil;
import frc.robot.lib.phoenixpro.TalonConfigHelper;
import frc.robot.subsystems.arm.Arm.GameObjectType;

public class GripperIOFalcon implements GripperIO {
    private final TalonFX mGripperMotor;
    private final TalonFXConfiguration mGripperConfig;
    private final DutyCycleOut mGripperControl;

    private final StatusSignal<Double> mGripperSpeed;
    private final StatusSignal<Double> mGripperSupplyCurrent;
    private final StatusSignal<Double> mGripperMotorTemp;
    private final StatusSignal<ForwardLimitValue> mGripperForwardLimit;
    private final StatusSignal<ReverseLimitValue> mGripperReverseLimit;

    private boolean mConeMode;

    public GripperIOFalcon() {
        mGripperConfig = TalonConfigHelper.getBaseConfig();
        mGripperConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        mGripperConfig.CurrentLimits.StatorCurrentLimit = 40;
        mGripperConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        mGripperConfig.CurrentLimits.SupplyCurrentLimit = 20;

        mGripperConfig.HardwareLimitSwitch.ForwardLimitEnable = false;
        mGripperConfig.HardwareLimitSwitch.ReverseLimitEnable = false;

        mGripperMotor = new TalonFX(40, "canivore");
        PhoenixProUtil.checkErrorAndRetry(() -> mGripperMotor.getConfigurator().apply(mGripperConfig));

        mGripperControl = new DutyCycleOut(0, true, false, false, false);

        mGripperSpeed = mGripperMotor.getVelocity();
        mGripperSupplyCurrent = mGripperMotor.getSupplyCurrent();
        mGripperMotorTemp = mGripperMotor.getDeviceTemp();
        mGripperForwardLimit = mGripperMotor.getForwardLimit();
        mGripperReverseLimit = mGripperMotor.getReverseLimit();

        mConeMode = false;
    }

    @Override
    public void updateInputs(GripperIOInputs inputs) {
        mGripperSpeed.refresh();
        mGripperSupplyCurrent.refresh();
        mGripperMotorTemp.refresh();
        mGripperForwardLimit.refresh();
        mGripperReverseLimit.refresh();

        inputs.motorSpeedRotationsPerSecond = mGripperSpeed.getValue();
        inputs.suppliedCurrentAmps = mGripperSupplyCurrent.getValue();
        inputs.hottestMotorTempCelsius = mGripperMotorTemp.getValue();

        inputs.cubeInIntake = mGripperReverseLimit.getValue() == ReverseLimitValue.ClosedToGround;
        inputs.coneInIntake = mGripperForwardLimit.getValue() == ForwardLimitValue.ClosedToGround;
    }

    @Override
    public void updateOutputs() {
        mGripperMotor.setControl(mGripperControl);
    }

    @Override
    public void setGameObject(GameObjectType object) {
        mConeMode = (object == GameObjectType.CONE);
    }

    @Override
    public void setMotor(double throttle) {
        throttle *= (mConeMode ? -1.0 : 1.0);
        mGripperControl.Output = throttle;
    }
}
