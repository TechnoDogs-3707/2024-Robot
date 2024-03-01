package frc.robot.subsystems.tilt;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import frc.robot.Constants;

public class TiltIOSim implements TiltIO {
    private final TrapezoidProfile.Constraints kConstraints;

    private TrapezoidProfile.State mLastState;
    private TrapezoidProfile.State mGoalState;

    private TrapezoidProfile mEstimator;

    public TiltIOSim() {
        kConstraints = new Constraints(Constants.ShooterTilt.kMagicVel, Constants.ShooterTilt.kMagicAccel);
        mLastState = new State();
        mGoalState = new State();

        mEstimator = new TrapezoidProfile(kConstraints);
    }

    @Override
    public void updateInputs(TiltIOInputs inputs) {
        inputs.tiltRotations = mLastState.position;
        inputs.tiltVelocityRotPerSec = mLastState.velocity;
    }

    @Override
    public void updateOutputs() {
        mLastState = mEstimator.calculate(Constants.loopPeriodSecs, mLastState, mGoalState);
    }

    @Override
    public void setTiltTarget(double tiltTargetRotations) {
        mGoalState.position = tiltTargetRotations;
    }
}
