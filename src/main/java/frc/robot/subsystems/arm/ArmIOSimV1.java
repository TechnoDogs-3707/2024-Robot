package frc.robot.subsystems.arm;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.*;
import frc.robot.Constants;

public class ArmIOSimV1 implements ArmIO {
    private final TrapezoidProfile.Constraints kTiltConstraints;
    private final TrapezoidProfile.Constraints kExtendConstraints;
    private final TrapezoidProfile.Constraints kWristConstraints;

    private TrapezoidProfile.State mTiltLastState;
    private TrapezoidProfile.State mExtendLastState;
    private TrapezoidProfile.State mWristLastState;

    private TrapezoidProfile.State mTiltGoalState;
    private TrapezoidProfile.State mExtendGoalState;
    private TrapezoidProfile.State mWristGoalState;

    private TrapezoidProfile mTiltEstimator;
    private TrapezoidProfile mExtendEstimator;
    private TrapezoidProfile mWristEstimator;

    public ArmIOSimV1() {
        kTiltConstraints = new Constraints(0.1875, 0.75);
        kExtendConstraints = new Constraints(1.5, 4);
        kWristConstraints = new Constraints(1.75, 3);

        mTiltLastState = new State();
        mExtendLastState = new State();
        mWristLastState = new State();

        mTiltGoalState = new State();
        mExtendGoalState = new State();
        mWristGoalState = new State();

        mTiltEstimator = new TrapezoidProfile(kTiltConstraints);
        mExtendEstimator = new TrapezoidProfile(kExtendConstraints);
        mWristEstimator = new TrapezoidProfile(kWristConstraints);
    }

    @Override
    public void updateInputs(ArmIOInputs inputs) {
        inputs.tiltRotations = mTiltLastState.position;
        inputs.tiltVelocityRotPerSec = mTiltLastState.velocity;
        
        inputs.extendMeters = mExtendLastState.position;
        inputs.extendVelocityMetersPerSec = mExtendLastState.velocity;
        
        inputs.wristRotations = mWristLastState.position;
        inputs.wristVelocityRotPerSec = mWristLastState.velocity;
    }

    @Override
    public void updateOutputs() {
        // mTiltLastState = mTiltEstimator.calculate(Constants.loopPeriodSecs, mTiltGoalState, mTiltLastState);
        // mExtendLastState = mExtendEstimator.calculate(Constants.loopPeriodSecs, mExtendGoalState, mExtendLastState);
        // mWristLastState = mWristEstimator.calculate(Constants.loopPeriodSecs, mWristGoalState, mWristLastState);
    
        mTiltLastState = mTiltEstimator.calculate(Constants.loopPeriodSecs, mTiltLastState, mTiltGoalState);
        mExtendLastState = mExtendEstimator.calculate(Constants.loopPeriodSecs, mExtendLastState, mExtendGoalState);
        mWristLastState = mWristEstimator.calculate(Constants.loopPeriodSecs, mWristLastState, mWristGoalState);
    }

    @Override
    public void setTiltTarget(double rotations) {
        mTiltGoalState.position = rotations;
    }

    @Override
    public void setExtendTarget(double meters) {
        mExtendGoalState.position = meters;
    }

    @Override
    public void setWristTarget(double rotations) {
        mWristGoalState.position = rotations;
    }
}