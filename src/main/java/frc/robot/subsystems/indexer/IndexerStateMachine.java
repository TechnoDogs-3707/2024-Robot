package frc.robot.subsystems.indexer;

import static frc.robot.subsystems.indexer.IndexerConstants.*;

public class IndexerStateMachine {
    public enum IndexerWantedAction {
        /**
         * Does not run the indexer motors under any condition.
         */
        OFF,

        /**
         * Will run the indexer motors until the first banner sensor is triggered.
         * If the second banner sensor is triggered, the indexer will reverse slowly
         * until only the first sensor is triggered.
         */
        INTAKE,

        /**
         * Runs the indexer motors forwards until 0.25 seconds after the second banner sensor
         * is no longer triggered.
         */
        SCORE,

        /**
         * Runs the indexer motors in reverse for a long as this mode is set, since we can't
         * tell for sure if the gamepiece has been dropped yet.
         */
        REVERSE
    }

    public enum IndexerSystemState {
        /**
         * The default state of the indexer. No motors are running.
         */
        OFF_EMPTY,

        /**
         * Motor is running, and the first banner sensor is not triggered.
         */
        INTAKING,

        /**
         * Both banner sensors are triggered, and the motor is running slowly in reverse.
         */
        OVERFED,

        /**
         * Only the first banner sensor is triggered, and the motor is stopped.
         */
        OFF_FULL,

        /**
         * The motor is feeding forwards.
         */
        SCORING,

        /**
         * The motor is feeding backwards.
         */
        REVERSING,
    }

    private IndexerWantedAction mWantedAction = IndexerWantedAction.OFF;
    private IndexerSystemState mSystemState = IndexerSystemState.OFF_EMPTY;
    // private double mStateStartTime = Timer.getFPGATimestamp();

    public void setWantedAction(IndexerWantedAction wantedAction) {
        if (wantedAction != mWantedAction) {
            mWantedAction = wantedAction;
            // mStateStartTime = Timer.getFPGATimestamp();
        }
    }

    public IndexerSystemState getSystemState() {
        return mSystemState;
    }

    protected double update(boolean firstBannerTriggered, boolean secondBannerTriggered) {
        // double timeInState = Timer.getFPGATimestamp() - mStateStartTime;

        double desiredThrottle = 0;

        switch (mWantedAction) {
            case OFF:
                if (!firstBannerTriggered && !secondBannerTriggered) {
                    mSystemState = IndexerSystemState.OFF_EMPTY;
                } else {
                    mSystemState = IndexerSystemState.OFF_FULL;
                }
                break;
            case INTAKE:
                if (firstBannerTriggered && !secondBannerTriggered) {
                    mSystemState = IndexerSystemState.OFF_FULL;
                } else if (firstBannerTriggered && secondBannerTriggered) {
                    mSystemState = IndexerSystemState.OVERFED;
                } else {
                    mSystemState = IndexerSystemState.INTAKING;
                }
                break;
            case SCORE:
                if (firstBannerTriggered || secondBannerTriggered) {
                    mSystemState = IndexerSystemState.SCORING;
                } else {
                    mSystemState = IndexerSystemState.OFF_EMPTY;
                }
                break;
            case REVERSE:
                mSystemState = IndexerSystemState.REVERSING;
                break;
            default:
                mSystemState = IndexerSystemState.OFF_EMPTY;
                break;
        }

        switch (mSystemState) {
            case OFF_EMPTY:
                desiredThrottle = kIdleThrottle;
                break;
            case INTAKING:
                desiredThrottle = kIntakingThrottle;
                break;
            case OVERFED:
                desiredThrottle = kOverfedThrottle;
                break;
            case OFF_FULL:
                desiredThrottle = kIdleThrottle;
                break;
            case SCORING:
                desiredThrottle = kScoringThrottle;
                break;
            case REVERSING:
                desiredThrottle = kReversingThrottle;
                break;
            default:
                break;
        }

        return desiredThrottle;
    }
}
