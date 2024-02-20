package frc.robot.subsystems.arm;

import edu.wpi.first.wpilibj.Timer;

import static frc.robot.Constants.ArmSubsystem.Intake.*;

public class ArmIntakeStateMachine {
    public enum ArmIntakeWantedAction {
        OFF,
        INTAKE_PARTIAL,
        INTAKE_HANDOFF,
        INTAKE_CONSTANT,
        REVERSE
    }

    public enum ArmIntakeSystemState {
        OFF,
        INTAKE_PARTIAL_EMPTY,
        INTAKE_PARTIAL_FULL,
        INTAKE_HANDOFF_FULL,
        INTAKE_HANDOFF_EMPTY,
        INTAKE_CONSTANT,
        INTAKE_REVERSE
    }

    private ArmIntakeWantedAction mWantedAction = ArmIntakeWantedAction.OFF;
    private ArmIntakeSystemState mSystemState = ArmIntakeSystemState.OFF;
    private double mStateStartTime = Timer.getFPGATimestamp();

    public void setWantedAction(ArmIntakeWantedAction wantedAction) {
        if (wantedAction != mWantedAction) {
            mWantedAction = wantedAction;
            mStateStartTime = Timer.getFPGATimestamp();
        }
    }

    public ArmIntakeSystemState getSystemState() {
        return mSystemState;
    }

    public ArmIntakeWantedAction getWantedAction() {
        return mWantedAction;
    }

    public double update(boolean intakeBeamBreakTriggered) {
        double timeInState = Timer.getFPGATimestamp() - mStateStartTime;

        double desiredThrottle = 0;

        switch (mWantedAction) {
            case OFF:
                mSystemState = ArmIntakeSystemState.OFF;
                break;
            case INTAKE_PARTIAL:
                if (intakeBeamBreakTriggered) {
                    mSystemState = ArmIntakeSystemState.INTAKE_PARTIAL_FULL;
                } else {
                    mSystemState = ArmIntakeSystemState.INTAKE_PARTIAL_EMPTY;
                }
                break;
            case INTAKE_HANDOFF:
                if (intakeBeamBreakTriggered) {
                    mSystemState = ArmIntakeSystemState.INTAKE_HANDOFF_FULL;
                } else {
                    mSystemState = ArmIntakeSystemState.INTAKE_HANDOFF_EMPTY;
                }
                break;
            case INTAKE_CONSTANT:
                mSystemState = ArmIntakeSystemState.INTAKE_CONSTANT;
                break;
            case REVERSE:
                mSystemState = ArmIntakeSystemState.INTAKE_REVERSE;
                break;
            default:
                mSystemState = ArmIntakeSystemState.OFF;
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
