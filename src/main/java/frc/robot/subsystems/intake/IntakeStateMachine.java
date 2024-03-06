package frc.robot.subsystems.intake;

import static frc.robot.Constants.Intake.*;

public class IntakeStateMachine {
    public enum IntakeWantedAction {
        OFF,
        INTAKE_PARTIAL,
        INTAKE_HANDOFF,
        INTAKE_CONSTANT,
        REVERSE
    }

    public enum IntakeSystemState {
        OFF,
        INTAKE_PARTIAL_EMPTY,
        INTAKE_PARTIAL_FULL,
        INTAKE_HANDOFF_FULL,
        INTAKE_HANDOFF_EMPTY,
        INTAKE_CONSTANT,
        INTAKE_REVERSE
    }

    private IntakeWantedAction mWantedAction = IntakeWantedAction.OFF;
    private IntakeSystemState mSystemState = IntakeSystemState.OFF;
    // private double mStateStartTime = Timer.getFPGATimestamp();

    public void setWantedAction(IntakeWantedAction wantedAction) {
        if (wantedAction != mWantedAction) {
            mWantedAction = wantedAction;
            // mStateStartTime = Timer.getFPGATimestamp();
        }
    }

    public IntakeSystemState getSystemState() {
        return mSystemState;
    }

    public IntakeWantedAction getWantedAction() {
        return mWantedAction;
    }

    public double update(boolean intakeBeamBreakTriggered) {
        // double timeInState = Timer.getFPGATimestamp() - mStateStartTime;

        double desiredThrottle = 0;

        switch (mWantedAction) {
            case OFF:
                mSystemState = IntakeSystemState.OFF;
                break;
            case INTAKE_PARTIAL:
                if (intakeBeamBreakTriggered) {
                    mSystemState = IntakeSystemState.INTAKE_PARTIAL_FULL;
                } else {
                    mSystemState = IntakeSystemState.INTAKE_PARTIAL_EMPTY;
                }
                break;
            case INTAKE_HANDOFF:
                if (intakeBeamBreakTriggered) {
                    mSystemState = IntakeSystemState.INTAKE_HANDOFF_FULL;
                } else {
                    mSystemState = IntakeSystemState.INTAKE_HANDOFF_EMPTY;
                }
                break;
            case INTAKE_CONSTANT:
                mSystemState = IntakeSystemState.INTAKE_CONSTANT;
                break;
            case REVERSE:
                mSystemState = IntakeSystemState.INTAKE_REVERSE;
                break;
            default:
                mSystemState = IntakeSystemState.OFF;
                break;
        }

        // TODO: move these values to constants
        switch (mSystemState) {
            case OFF:
                desiredThrottle = kIdleThrottle;
                break;
            case INTAKE_PARTIAL_EMPTY:
                desiredThrottle = kPartialIntakeThrottle;
                break;
            case INTAKE_PARTIAL_FULL:
                desiredThrottle = kIdleThrottle;
                break;
            case INTAKE_HANDOFF_FULL:
                desiredThrottle = kHandoffThrottle;
                break;
            case INTAKE_HANDOFF_EMPTY:
                desiredThrottle = kIdleThrottle;
                break;
            case INTAKE_CONSTANT:
                desiredThrottle = kConstantThrottle;
                break;
            case INTAKE_REVERSE:
                desiredThrottle = kReverseThrottle;
                break;
            default:
                desiredThrottle = kIdleThrottle;
                break;
        }

        return desiredThrottle;
    }
}
