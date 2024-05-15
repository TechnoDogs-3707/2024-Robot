package frc.robot.subsystems.shooterTilt;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import frc.robot.Constants;

public class TiltIOSim implements ShooterTiltIO {
    private final TrapezoidProfile.Constraints kConstraints;

    private TrapezoidProfile.State mLastState;
    private TrapezoidProfile.State mGoalState;

    private TrapezoidProfile mEstimator;

    public TiltIOSim() {
        kConstraints = new Constraints(ShooterTiltConstants.kMagicVel, ShooterTiltConstants.kMagicAccel);
        mLastState = new State();
        mGoalState = new State();

        mEstimator = new TrapezoidProfile(kConstraints);
    }

    @Override
    public void updateInputs(ShooterTiltIOInputs inputs) {
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
