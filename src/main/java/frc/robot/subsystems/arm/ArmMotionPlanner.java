package frc.robot.subsystems.arm;

import java.util.LinkedList;
import java.util.Optional;

// import static frc.robot.Constants.ArmSubsystem.*;

public class ArmMotionPlanner {
    private static final double kMinTiltForExtension = 0.02; // Minimum tilt to allow extension
    private static final double kMaxExtensionWhileStowed = 0.02; // Maximum extension to allow tilting below min tilt
    // private static final double kScoringWaitTime = 0.1; // how long to stay at scoring position

    protected LinkedList<ArmState> mIntermediateStateQueue = new LinkedList<>();
    private Optional<ArmState> mCurrentCommandedState = Optional.empty();
    private Optional<ArmState> mDesiredState = Optional.empty();
    // private double mStartedWaitingTimestamp = 0.0;
    // private double mScoringOffset = 0.0;

    // public synchronized void setScoringOffset(double x) {
    //     mScoringOffset = x;
    // }

    public synchronized void setDesiredState(ArmState desiredState, ArmState currentState) {
        // Optional<ArmState> mLastDesiredState = mDesiredState;
        mDesiredState = Optional.of(new ArmState(desiredState));

        // Everything beyond this is probably do-able; clear queue
        mIntermediateStateQueue.clear();

        // Which actions need conservative tolerances
        // boolean isConservative = mDesiredState.get().action == ArmState.Action.INTAKING
                // || mDesiredState.get().action == ArmState.Action.SCORING;

        // double tiltAllowableError = (isConservative ? Tilt.kConservativeAllowableError : Tilt.kLiberalAllowableError);
        // double extendAllowableError = (isConservative ? Extend.kConservativeAllowableError : Extend.kLiberalAllowableError);
        // double wristAllowableError = (isConservative ? Wrist.kConservativeAllowableError : Wrist.kLiberalAllowableError);

        /* 
         *  First, we check for invalid end states that meet any of the following conditions.
         *  - Any joint position exceeds its individual travel limits
         *  - Combined position that violates frame extension or height limits
         *  - Combined position that intersects with the bumpers or rear superstructure plate
         * 
         *  Once we have confirmed that the end position is valid, we must now compute a series of
         *  intermediate positions that will safely and quickly get us between any two positions.
         * 
         * For positions where J1 and J2 moves through the range where the total length would exceed the
         * height limit, we need to add an intermediate state with J1 and J2 at the closest position either side
         * that will not exceed this limit. 
         */

        // all other conditions, go straight to position
        mIntermediateStateQueue.add(desiredState);
    }

    public void reset() {
        mIntermediateStateQueue.clear();
        mCurrentCommandedState = Optional.empty();
    }

    public boolean isFinished() {
        return mCurrentCommandedState.isEmpty() && mIntermediateStateQueue.isEmpty();
    }

    public int getRemainingStates() {
        return mIntermediateStateQueue.size();
    }

    public ArmState update(ArmState currentState) {

        if (mCurrentCommandedState.isEmpty() && !mIntermediateStateQueue.isEmpty()) {
            mCurrentCommandedState = Optional.of(mIntermediateStateQueue.remove());
        }

        if (mCurrentCommandedState.isPresent() && mCurrentCommandedState.get().isInRange(currentState)) {
            mCurrentCommandedState = Optional.empty(); // Commanded state will be updated at beginning of next update call
        }

        return mCurrentCommandedState.orElse(mDesiredState.orElse(currentState));
    }
}
