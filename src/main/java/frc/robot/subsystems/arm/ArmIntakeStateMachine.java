package frc.robot.subsystems.arm;

import edu.wpi.first.wpilibj.Timer;

import static frc.robot.Constants.ArmSubsystem.Intake.*;

public class ArmIntakeStateMachine {
    public enum WantedAction {
        OFF,
        INTAKE_PARTIAL,
        INTAKE_HANDOFF,
        INTAKE_CONSTANT,
        REVERSE
    }

    public enum SystemState {
        OFF,
        INTAKE_PARTIAL_EMPTY,
        INTAKE_PARTIAL_FULL,
        INTAKE_HANDOFF_FULL,
        INTAKE_HANDOFF_EMPTY,
        INTAKE_CONSTANT,
        INTAKE_REVERSE
    }

    private WantedAction mWantedAction = WantedAction.OFF;
    private SystemState mSystemState = SystemState.OFF;
    private double mStateStartTime = Timer.getFPGATimestamp();

    public void setWantedAction(WantedAction wantedAction) {
        if (wantedAction != mWantedAction) {
            mWantedAction = wantedAction;
            mStateStartTime = Timer.getFPGATimestamp();
        }
    }

    public SystemState getSystemState() {
        return mSystemState;
    }

    public WantedAction getWantedAction() {
        return mWantedAction;
    }

    public double update(boolean intakeBeamBreakTriggered) {
        double timeInState = Timer.getFPGATimestamp() - mStateStartTime;

        double desiredThrottle = 0;

        switch (mWantedAction) {
            case OFF:
                mSystemState = SystemState.OFF;
                break;
            case INTAKE_PARTIAL:
                if (intakeBeamBreakTriggered) {
                    mSystemState = SystemState.INTAKE_PARTIAL_FULL;
                } else {
                    mSystemState = SystemState.INTAKE_PARTIAL_EMPTY;
                }
                break;
            case INTAKE_HANDOFF:
                if (intakeBeamBreakTriggered) {
                    mSystemState = SystemState.INTAKE_HANDOFF_FULL;
                } else {
                    mSystemState = SystemState.INTAKE_HANDOFF_EMPTY;
                }
                break;
            case INTAKE_CONSTANT:
                mSystemState = SystemState.INTAKE_CONSTANT;
                break;
            case REVERSE:
                mSystemState = SystemState.INTAKE_REVERSE;
                break;
            default:
                mSystemState = SystemState.OFF;
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
