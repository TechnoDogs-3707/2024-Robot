package frc.robot.subsystems.intake;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.lib.phoenixpro.PhoenixProUtil;
import frc.robot.lib.phoenixpro.TalonFXConfigHelper;

public class IntakeIOTalonFX implements IntakeIO {
    private final TalonFX mIntakeMotorMaster;

    private final TalonFXConfiguration mIntakeConfig;

    private final DutyCycleOut mIntakeControlMaster;

    private StatusSignal<Double> intakeMasterVelocity;
    private StatusSignal<Double> intakeMasterSuppliedCurrent;
    private StatusSignal<Double> intakeMasterTempCelsius;

    private final DigitalInput mIntakeSensor;

    public IntakeIOTalonFX() {
        mIntakeMotorMaster = new TalonFX(IntakeConstants.kMasterMotorID, IntakeConstants.kMotorBus);

        mIntakeConfig = TalonFXConfigHelper.DefaultConfigs.getBaseConfig(); //TODO: set up intake config
        mIntakeConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        mIntakeControlMaster = new DutyCycleOut(0, false, false, false, false);

        mIntakeSensor = new DigitalInput(5);

        intakeMasterVelocity = mIntakeMotorMaster.getVelocity();
        intakeMasterSuppliedCurrent = mIntakeMotorMaster.getSupplyCurrent();
        intakeMasterTempCelsius = mIntakeMotorMaster.getDeviceTemp();

        PhoenixProUtil.checkErrorAndRetry(() -> mIntakeMotorMaster.getConfigurator().apply(mIntakeConfig));
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.intakeVelocityRotPerSec = intakeMasterVelocity.getValue();
        inputs.intakeSuppliedCurrentAmps = intakeMasterSuppliedCurrent.getValue();
        inputs.intakeHottestTempCelsius = intakeMasterTempCelsius.getValue();

        inputs.intakeBeamBreakTriggered = !mIntakeSensor.get();
    }

    @Override
    public void setIntakeThrottle(double throttle) {
        mIntakeControlMaster.Output = throttle;
    }

    @Override
    public void updateOutputs() {
        mIntakeMotorMaster.setControl(mIntakeControlMaster);
    }
}
