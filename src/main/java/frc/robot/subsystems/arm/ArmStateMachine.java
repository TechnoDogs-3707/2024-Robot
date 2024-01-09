package frc.robot.subsystems.arm;

import frc.robot.lib.util.Util;

public class ArmStateMachine {
    public enum WantedAction {
        IDLE,
        GO_TO_POSIION
    }

    public enum SystemState {
        HOLDING_POSITION,
        MOVING_TO_POSITION
    }

    private SystemState mSystemState = SystemState.HOLDING_POSITION;

    private ArmState mCommandedState = new ArmState();
    private ArmState mLastDesireEndState = new ArmState();
    private ArmState mDesiredEndState = new ArmState();

    private ArmMotionPlanner mPlanner = new ArmMotionPlanner();

    public synchronized void setScoringPosition(ArmState scoringPosition) {
        mDesiredEndState = scoringPosition;
    }

    public synchronized boolean scoringPositionChanged() {
        boolean changed = !Util.epsilonEquals(mDesiredEndState.tilt, mLastDesireEndState.tilt)
                || !Util.epsilonEquals(mDesiredEndState.extend, mLastDesireEndState.extend)
                || !Util.epsilonEquals(mDesiredEndState.wrist, mLastDesireEndState.wrist)
                || mDesiredEndState.action != mLastDesireEndState.action;
        return changed;
    }

    public synchronized SystemState getSystemState() {
        return mSystemState;
    }

    // public synchronized void setScoringOffset(double x) {mPlanner.setScoringOffset(x);}

    public synchronized ArmState update(double timestamp, WantedAction wantedAction,
                                                     ArmState currentState) {
        synchronized (ArmStateMachine.this) {
            SystemState newState;

            // Handle state transitions
            switch (mSystemState) {
                case HOLDING_POSITION:
                    newState = handleDefaultTransitions(wantedAction, currentState);
                    break;
                case MOVING_TO_POSITION:
                    newState = handleDefaultTransitions(wantedAction, currentState);
                    break;
                default:
                    System.out.println("Unexpected superstructure system state: " + mSystemState);
                    newState = mSystemState;
                    break;
            }

            if (newState != mSystemState) {
                mSystemState = newState;
            }

            // mCommandedState = mPlanner.update(currentState);
            return mCommandedState;
        }
    }

    private void updateMotionPlannerDesired(ArmState currentState) {
        mPlanner.setDesiredState(mDesiredEndState, currentState);
    }

    private SystemState handleDefaultTransitions(WantedAction wantedAction, ArmState currentState) {
        if (scoringPositionChanged()) {
            mLastDesireEndState = mDesiredEndState;
            updateMotionPlannerDesired(currentState);
        }
        return SystemState.MOVING_TO_POSITION;
    }
}
