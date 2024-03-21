package frc.robot.subsystems.armTilt;

import static frc.robot.subsystems.armTilt.ArmTiltConstants.*;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Constants.Mode;
import frc.robot.util.poofsUtils.PoofsUtil;

public class ArmTilt extends SubsystemBase {
    public enum ArmPositionPreset {
        STOWED(0.0);

        public double position;

        private ArmPositionPreset(double position) {
            this.position = position;
        }
    }

    private ArmTiltIO mIO;
    private ArmTiltIOInputsAutoLogged mInputs;

    private ArmPositionPreset mPositionPreset = ArmPositionPreset.STOWED;
    private boolean mUseManualThrottle = false;
    private double mManualThrottle = 0.0;

    private double mCurrentLimit = 30.0;
    private boolean mCurrentLimitSet = false;

    private boolean mWithinTolerance = false;

    public ArmTilt(ArmTiltIO io) {
        mIO = io;
        mInputs = new ArmTiltIOInputsAutoLogged();
    }

    @Override
    public void periodic() {
        // Update and process inputs
        mIO.updateInputs(mInputs);
        Logger.processInputs("ArmTilt", mInputs);

        // update sim current draw
        if (Constants.getMode() == Mode.SIM) {
            Robot.updateSimCurrentDraw("ArmTilt", mInputs.suppliedCurrentAmps);
        }

        // cache goal state and current limit, then record outputs. Also check if we are at target
        if (mCurrentLimitSet == false) {
            mIO.setTorqueLimit(mCurrentLimit);
            mCurrentLimitSet = true;
        }
        double positionTarget = mPositionPreset.position;
        mWithinTolerance = PoofsUtil.epsilonEquals(mInputs.rotations, positionTarget, ArmTiltConstants.kConservativeAllowableError);

        Logger.recordOutput("ArmTilt/TorqueLimitAmps", mCurrentLimit);
        Logger.recordOutput("ArmTilt/PositionPreset", mPositionPreset.name());
        Logger.recordOutput("ArmTilt/FinalPositionTarget", positionTarget);
        Logger.recordOutput("ArmTilt/UseOverrideThrottle", mUseManualThrottle);
        Logger.recordOutput("ArmTilt/OverrideThrottle", mManualThrottle);
        Logger.recordOutput("ArmTilt/WithinTolerance", mWithinTolerance);

        // set targets and update outputs
        mIO.enableManualThrottle(mUseManualThrottle);
        mIO.setManualThrottle(mManualThrottle);
        mIO.setPositionTarget(positionTarget);

        mIO.updateOutputs();
    }

    public void setPositionPreset(ArmPositionPreset positionPreset) {
        mPositionPreset = positionPreset;
        mWithinTolerance = false;
        mUseManualThrottle = false;
        mManualThrottle = 0.0;
    }

    public boolean atTarget() {
        return mWithinTolerance;
    }

    public Command setPositionBlockingCommand(ArmPositionPreset preset) {
        return setPositionBlockingCommand(() -> preset);
    }

    public Command setPositionBlockingCommand(Supplier<ArmPositionPreset> preset) {
        return runOnce(() -> setPositionPreset(preset.get())).andThen(
            Commands.waitUntil(this::atTarget)
        );
    }

    public Command setPositionCommand(ArmPositionPreset preset) {
        return setPositionCommand(() -> preset);
    }

    public Command setPositionCommand(Supplier<ArmPositionPreset> preset) {
        return runOnce(() -> setPositionPreset(preset.get()));
    }

    public void setManualThrottle(double manualThrottle) {
        mWithinTolerance = false;
        mUseManualThrottle = true;
        mManualThrottle = manualThrottle;
    }

    public Command setManualThrottleCommand(Supplier<Double> manualThrottle) {
        return runOnce(() -> setManualThrottle(manualThrottle.get()));
    }

    public Command setManualThrottleCommand(Double manualThrottle) {
        return setManualThrottleCommand(() -> manualThrottle);
    }

    public void homeEncoder() {
        mIO.zeroPositon();
    }

    public Command homeEncoderCommand() {
        return setManualThrottleCommand(kHomingThrottle)
                .andThen(Commands.waitSeconds(kHomingMinTime))
                .andThen(Commands.waitUntil(() -> {
                    return getMotorVelocity() <= kHomingVelocityThreshold;
                }))
                .andThen(runOnce(this::homeEncoder))
                .finallyDo(() -> {
                    setManualThrottle(0.0);
                });
    }

    public double getMotorVelocity() {
        return mInputs.velocityRotPerSec;
    }
}
