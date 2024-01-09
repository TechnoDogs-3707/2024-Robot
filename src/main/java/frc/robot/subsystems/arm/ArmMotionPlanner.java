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

        // if current extension is above MEWS and current tilt is below MTFE, we are in an invalid state.
        // this should be corrected by tilting up to MTFE, then retracting to zero, then tilting to zero.
        // The current state variable can then be overridden to generate the next steps

        if (currentState.getExtend() > kMaxExtensionWhileStowed && currentState.getTilt() < kMinTiltForExtension) {
            // TODO: update with stow safe positions
            mIntermediateStateQueue.add(
                ArmState.generateWithFailsafeParameters(kMinTiltForExtension + 0.02, currentState.extend, currentState.wrist)
            );
            mIntermediateStateQueue.add(
                ArmState.generateWithFailsafeParameters(kMinTiltForExtension + 0.02, currentState.extend, 0)
            );
            mIntermediateStateQueue.add(
                ArmState.generateWithFailsafeParameters(kMinTiltForExtension + 0.02, 0, 0)
            );
            mIntermediateStateQueue.add(
                ArmState.generateWithFailsafeParameters(0, 0, 0)
            );
            currentState = new ArmState();
        }

        // if target extension is above MEWS and target tilt is below MTFE, we are targeting an invalid state.
        // this should be handled by reverting to stowed position.

        if (desiredState.extend >= kMaxExtensionWhileStowed && desiredState.tilt <= kMinTiltForExtension) {
            desiredState = new ArmState();
        }

        // if target extension is above MEWS and current tilt is below MTFE, we need to tilt first
        // not bothering with pre extending by like 2cm because the time gain is insignificant

        if (desiredState.extend >= kMaxExtensionWhileStowed && currentState.tilt <= kMinTiltForExtension) {
            mIntermediateStateQueue.add(new ArmState(desiredState.tilt, 0, 0));
            mIntermediateStateQueue.add(desiredState);
        } else

        // if target tilt is below MTFE and current extension is above MEWT, 
        // retract as much as possible, then tilt to final target and extend

        if (desiredState.tilt <= kMinTiltForExtension && currentState.extend >= kMaxExtensionWhileStowed) {
            mIntermediateStateQueue.add(new ArmState(currentState.tilt, desiredState.extend, desiredState.wrist));
            mIntermediateStateQueue.add(desiredState);
        } else 

        // if target tilt is not equal to current tilt, we should fully retract before we can tilt
        if (desiredState.tilt != currentState.tilt) {
            mIntermediateStateQueue.add(new ArmState(currentState.tilt, 0, 0));
            mIntermediateStateQueue.add(new ArmState(desiredState.tilt, 0, 0));
            mIntermediateStateQueue.add(desiredState);
        } else

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
