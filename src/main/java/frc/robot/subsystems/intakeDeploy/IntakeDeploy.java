package frc.robot.subsystems.intakeDeploy;

import static frc.robot.subsystems.intakeDeploy.IntakeDeployConstants.*;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Constants.Mode;
import frc.robot.util.poofsUtils.PoofsUtil;

public class IntakeDeploy extends SubsystemBase {
    public enum IntakePositionPreset {
        STOWED(0.3),
        EJECT(0.1875),
        AMP_SCORE(0.15),
        HANDOFF(-0.03),
        DEPLOYED(-0.1);

        public double position;

        private IntakePositionPreset(double position) {
            this.position = position;
        }
    }

    private IntakeDeployIO mIO;
    private IntakeDeployIOInputsAutoLogged mInputs;

    private IntakePositionPreset mPositionPreset = IntakePositionPreset.STOWED;
    private boolean mUseManualThrottle = false;
    private double mManualThrottle = 0.0;

    private double mCurrentLimit = 30.0;
    private boolean mCurrentLimitSet = false;

    private boolean mWithinTolerance = false;

    public IntakeDeploy(IntakeDeployIO io) {
        mIO = io;
        mInputs = new IntakeDeployIOInputsAutoLogged();
    }

    @Override
    public void periodic() {
        // Update and process inputs
        mIO.updateInputs(mInputs);
        Logger.processInputs("IntakeDeploy", mInputs);

        // update sim current draw
        if (Constants.getMode() == Mode.SIM) {
            Robot.updateSimCurrentDraw("IntakeDeploy", mInputs.suppliedCurrentAmps);
        }

        // cache goal state and current limit, then record outputs. Also check if we are at target
        if (mCurrentLimitSet == false) {
            mIO.setTorqueLimit(mCurrentLimit);
            mCurrentLimitSet = true;
        }
        double positionTarget = mPositionPreset.position;
        mWithinTolerance = PoofsUtil.epsilonEquals(mInputs.rotations, positionTarget, IntakeDeployConstants.kConservativeAllowableError);

        Logger.recordOutput("IntakeDeploy/TorqueLimitAmps", mCurrentLimit);
        Logger.recordOutput("IntakeDeploy/PositionPreset", mPositionPreset.name());
        Logger.recordOutput("IntakeDeploy/FinalPositionTarget", positionTarget);
        Logger.recordOutput("IntakeDeploy/UseOverrideThrottle", mUseManualThrottle);
        Logger.recordOutput("IntakeDeploy/OverrideThrottle", mManualThrottle);
        Logger.recordOutput("IntakeDeploy/WithinTolerance", mWithinTolerance);

        // set targets and update outputs
        mIO.enableManualThrottle(mUseManualThrottle);
        mIO.setManualThrottle(mManualThrottle);
        mIO.setPositionTarget(positionTarget);

        mIO.updateOutputs();
    }

    public void setPositionPreset(IntakePositionPreset positionPreset) {
        mPositionPreset = positionPreset;
        mWithinTolerance = false;
        mUseManualThrottle = false;
        mManualThrottle = 0.0;
    }

    public boolean atTarget() {
        return mWithinTolerance;
    }

    public Command setPositionBlockingCommand(IntakePositionPreset preset) {
        return setPositionBlockingCommand(() -> preset);
    }

    public Command setPositionBlockingCommand(Supplier<IntakePositionPreset> preset) {
        return runOnce(() -> setPositionPreset(preset.get())).andThen(
            Commands.waitUntil(this::atTarget)
        );
    }

    public Command setPositionCommand(IntakePositionPreset preset) {
        return setPositionCommand(() -> preset);
    }

    public Command setPositionCommand(Supplier<IntakePositionPreset> preset) {
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
