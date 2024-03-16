package frc.robot.subsystems.intakeDeploy;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import frc.robot.Constants;

import static frc.robot.subsystems.intakeDeploy.IntakeDeployConstants.*;

public class IntakeDeployIOSim implements IntakeDeployIO{
    private final TrapezoidProfile.Constraints kConstraints;

    private TrapezoidProfile.State mLastState;
    private TrapezoidProfile.State mGoalState;

    private TrapezoidProfile mEstimator;

    public IntakeDeployIOSim() {
        kConstraints = new Constraints(kMagicVel, kMagicAccel);
        mLastState = new State();
        mGoalState = new State();

        mEstimator = new TrapezoidProfile(kConstraints);
    }

    @Override
    public void updateInputs(IntakeDeployIOInputs inputs) {
        inputs.rotations = mLastState.position;
        inputs.velocityRotPerSec = mLastState.velocity;
    }

    @Override
    public void updateOutputs() {
        mLastState = mEstimator.calculate(Constants.loopPeriodSecs, mLastState, mGoalState);
    }

    @Override
    public void setPositionTarget(double tiltTargetRotations) {
        mGoalState.position = tiltTargetRotations;
    }
}
