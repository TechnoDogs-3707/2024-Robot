package frc.robot.subsystems.arm;

import java.util.LinkedList;
import java.util.NoSuchElementException;
import java.util.Optional;

import frc.robot.lib.util.Util;
import frc.robot.subsystems.arm.Arm.GoalState;

import static frc.robot.Constants.ArmSubsystem.*;

public class ArmMotionPlanner {
    private static final double kMaxJ1ForFullJ2Travel = 0.0;
    private static final double kMinJ2ForFullJ1Travel = 0.0;
    private static final double kMaxSafeJ1Position = 0.0;
    // private static final double kScoringWaitTime = 0.1; // how long to stay at scoring position

    protected LinkedList<ArmState> mIntermediateStateQueue = new LinkedList<>();
    private Optional<ArmState> mCurrentCommandedState = Optional.empty();
    private Optional<ArmState> mDesiredState = Optional.empty();
    // private double mStartedWaitingTimestamp = 0.0;
    // private double mScoringOffset = 0.0;

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
         *  - TODO: Combined position intersects bumpers or other robot parts
         * 
         *  Once we have confirmed that the end position is valid, we must now compute a series of
         *  intermediate positions that will safely and quickly get us between any two positions.
         */
        // boolean j1TargetOutsideLimits = !Util.inRange(desiredState.j1, J1.kMinTargetPosition, J1.kMaxTargetPosition);
        // boolean j2TargetOutsideLimits = !Util.inRange(desiredState.j2, J2.kMinTargetPosition, J2.kMaxTargetPosition);
        // if (j1TargetOutsideLimits || j2TargetOutsideLimits) {
        //     mDesiredState = Optional.of(GoalState.STOW.state); // override desired state to STOW.
        // }

        /*
         * Second, if the desired state has J1 above the highest allowable height for full J2 travel,
         * and the current state has J1 below the highest allowable height for full J2 travel, we need 
         * to tilt J2 to the minimum tilt for full J1 motion range, then continue.
         */
        // if (desiredState.j1 > kMaxJ1ForFullJ2Travel && currentState.j1 < kMaxJ1ForFullJ2Travel) {
        //     mIntermediateStateQueue.add(new ArmState(kMaxJ1ForFullJ2Travel, kMinJ2ForFullJ1Travel));
        // }

        /*
         * Third, if the desired, but not current J1 is above the maximum safe J1, we should rotate to 
         * the maximum safe J1 and target J2 position.
         */
        // if (desiredState.j1 > kMaxSafeJ1Position && currentState.j1 <= kMaxSafeJ1Position) {
        //     mIntermediateStateQueue.add(new ArmState(kMaxSafeJ1Position, desiredState.j2));
        // }

        /*
         * Fourth, if the current, but not desired J1 is above the maximum safe J1, we should rotate to 
         * the maximum safe J1.
         */
        // if (currentState.j1 > kMaxSafeJ1Position && desiredState.j1 <= kMaxSafeJ1Position) {
        //     mIntermediateStateQueue.add(new ArmState(kMaxSafeJ1Position, getLastIntermediateStateOrElse(currentState).j2));
        // }

        /*
         * Fifth, if the current and desired J1 is above the safe limit, then rotate to max safe J1, then rotate to
         * target J2.
         */
        // if (currentState.j1 > kMaxSafeJ1Position && desiredState.j1 > kMaxSafeJ1Position) {
        //     mIntermediateStateQueue.add(new ArmState(kMaxSafeJ1Position, getLastIntermediateStateOrElse(currentState).j2));
        //     mIntermediateStateQueue.add(new ArmState(kMaxSafeJ1Position, desiredState.j2));
        // }

        // all other conditions, go straight to position
        mIntermediateStateQueue.add(desiredState);
    }

    /**
     * Get the last element in the intermediate state list if present, otherwise return Other argument.
     * @param other The value to be returned if last intermediate state is empty.
     * @return The last intermediate state or the Other argument.
     */
    private ArmState getLastIntermediateStateOrElse(ArmState other) {
        try {
            ArmState realValue = mIntermediateStateQueue.getLast();
            return realValue;
        } catch (NoSuchElementException e) {
            return other;
        }
    }

    /**
     * Resets the arm motion planner by clearing the intermediate state queue and setting
     * the current commanded state to empty.
     */
    public void reset() {
        mIntermediateStateQueue.clear();
        mCurrentCommandedState = Optional.empty();
    }

    /**
     * Determines if the motion planner has reached its desired state by checking that the current
     * commanded state and intermediate state queue are both empty.
     * @return True if the motion planner is finished.
     */
    public boolean isFinished() {
        return mCurrentCommandedState.isEmpty() && mIntermediateStateQueue.isEmpty();
    }

    /**
     * Gets the number of remaining states in the motion planner's queue.
     * @return the length of the intermediate state queue.
     */
    public int getRemainingStates() {
        return mIntermediateStateQueue.size();
    }

    /**
     * Get the next state for the arm to rotate to. If the desired state is not present,
     * which may occur if the motion planner is finished, we will return the current state.
     * @param currentState The current state of the arm.
     * @return The next state, which may be the current state.
     */
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
