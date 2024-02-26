package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.phoenixpro.PhoenixProUtil;
import frc.robot.lib.phoenixpro.TalonFXConfigHelper;

public class Climb extends SubsystemBase {
    private final TalonFX mLeftClimb;
    private final TalonFX mRightClimb;

    private final TalonFXConfiguration mConfig;

    private final Supplier<Double> mJoystickSupplier;

    public Climb(Supplier<Double> joystickSupplier) {
        mLeftClimb = new TalonFX(50, "canivore");
        mRightClimb = new TalonFX(51, "canivore");

        mConfig = TalonFXConfigHelper.getBaseConfig();
        mConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        mConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        mConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 2;
        mConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        mConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 60;
        mConfig.MotorOutput.DutyCycleNeutralDeadband = 0.05;

        final TalonFXConfiguration mRightConfig = TalonFXConfigHelper.getBaseConfig();
        mRightConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        mRightConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        mRightConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 2;
        mRightConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        mRightConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 60;
        mRightConfig.MotorOutput.DutyCycleNeutralDeadband = 0.05;

        PhoenixProUtil.checkErrorAndRetry(() -> mLeftClimb.getConfigurator().apply(mConfig));
        PhoenixProUtil.checkErrorAndRetry(() -> mRightClimb.getConfigurator().apply(mRightConfig));

        mJoystickSupplier = joystickSupplier;
    }

    @Override
    public void periodic() {
        var val = mJoystickSupplier.get();
        mLeftClimb.set(-val);
        mRightClimb.set(-val);

        var leftPosition = mLeftClimb.getPosition().refresh().getValue();
        var rightPositon = mRightClimb.getPosition().refresh().getValue();
    }
}
