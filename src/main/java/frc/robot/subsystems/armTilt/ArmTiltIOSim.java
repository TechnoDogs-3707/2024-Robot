package frc.robot.subsystems.armTilt;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import frc.robot.Constants;

import static frc.robot.subsystems.armTilt.ArmTiltConstants.*;

public class ArmTiltIOSim implements ArmTiltIO {
    private final TrapezoidProfile.Constraints kConstraints;

    private TrapezoidProfile.State mLastState;
    private TrapezoidProfile.State mGoalState;

    private TrapezoidProfile mEstimator;

    public ArmTiltIOSim() {
        kConstraints = new Constraints(kMagicVel, kMagicAccel);
        mLastState = new State();
        mGoalState = new State();

        mEstimator = new TrapezoidProfile(kConstraints);
    }

    @Override
    public void updateInputs(ArmTiltIOInputs inputs) {
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
