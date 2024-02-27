package frc.robot.subsystems.objectiveTracker;

import frc.robot.lib.leds.TimedLEDState;
import frc.robot.lib.util.VirtualSubsystem;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.leds.LED;

public class ObjectiveTracker extends VirtualSubsystem {
    private final Intake mIntake;
    private final Indexer mIndexer;

    public enum MasterObjective {
        NONE,
        INTAKE_GROUND,
        INTAKE_SOURCE_AUTOALIGN,
        SCORE_SPEAKER_AUTOALIGN,
        SCORE_SPEAKER_AUTOAIM,
        SCORE_AMP_AUTOALIGN,
        CLIMB
    }

    public enum IntakeGroundState {
        TO_INDEXER,
        TO_INTAKE
    }

    public enum AutoAlignIntakeState {
        DRIVING_TO_TARGET,
        ON_TARGET
    }

    public enum SpeakerAutoAimState {
        STABILIZING_TRAJECTORY,
        SCORE_RUNNING,
        SCORE_FINISHED
    }

    public enum AutoAlignScoreState {
        DRIVING_TO_TARGET,
        SCORE_RUNNING,
        SCORE_FINISHED
    }

    public enum ClimbingState {
        RAISING,
        RAISED,
        CLIMBING_DUTY_CYCLE,
        CLIMBING_MANUAL
    }

    private MasterObjective mMasterObjective = MasterObjective.NONE;
    private IntakeGroundState mIntakeGroundState = IntakeGroundState.TO_INDEXER;
    private AutoAlignIntakeState mAutoIntakeState = AutoAlignIntakeState.DRIVING_TO_TARGET;
    private SpeakerAutoAimState mAutoAimState = SpeakerAutoAimState.STABILIZING_TRAJECTORY;
    private AutoAlignScoreState mAutoAlignState = AutoAlignScoreState.DRIVING_TO_TARGET;
    private ClimbingState mClimbingState = ClimbingState.RAISING;

    public void setMasterObjective(MasterObjective masterObjective) {
        mMasterObjective = masterObjective;
    }

    public MasterObjective getMasterObjective() {
        return mMasterObjective;
    }

    public void setIntakeGroundState(IntakeGroundState intakeGroundState) {
        mIntakeGroundState = intakeGroundState;
    }

    public IntakeGroundState getIntakeGroundState() {
        return mIntakeGroundState;
    }

    public void setAutoIntakeState(AutoAlignIntakeState autoIntakeState) {
        mAutoIntakeState = autoIntakeState;
    }

    public AutoAlignIntakeState getAutoIntakeState() {
        return mAutoIntakeState;
    }

    public void setAutoAimState(SpeakerAutoAimState autoAimState) {
        mAutoAimState = autoAimState;
    }

    public SpeakerAutoAimState getAutoAimState() {
        return mAutoAimState;
    }

    public void setAutoAlignState(AutoAlignScoreState autoAlignState) {
        mAutoAlignState = autoAlignState;
    }

    public AutoAlignScoreState getAutoAlignState() {
        return mAutoAlignState;
    }

    public void setClimbingState(ClimbingState climbingState) {
        mClimbingState = climbingState;
    }

    public ClimbingState getClimbingState() {
        return mClimbingState;
    }

    public void handleNoneLEDs() {
        if (mIntake.hasNote()) {
            LED.setArmLEDState(TimedLEDState.StaticLEDState.kNoteInArm);
        } else if (mIndexer.hasNote()) {
            LED.setArmLEDState(TimedLEDState.StaticLEDState.kNoteInIndexer);
        } else {
        }
    }

    public void handleIntakeGroundLEDs() {
       switch (mIntakeGroundState) {
        case TO_INTAKE:
            LED.setArmLEDState(TimedLEDState.RSLBasedLEDState.kIntakingGroundToArm);
            break;
        case TO_INDEXER:
            LED.setArmLEDState(TimedLEDState.RSLBasedLEDState.kIntakingGroundToIndexer);
            break;
        default:
            break;
       }
    }

    public void handleIntakeSourceAutoAlignLEDs() {
        switch (mAutoIntakeState) {
            case DRIVING_TO_TARGET:
                // TODO: percent full based on distance
                break;
            case ON_TARGET:
                LED.setArmLEDState(TimedLEDState.BlinkingLEDState.kIntakeAutoAlignOnTarget);
                break;
            default:
                break;
        }
    }

    public void handleScoreSpeakerAutoAimLEDs() {
        switch (mAutoAimState) {
            case SCORE_FINISHED:
                LED.setArmLEDState(TimedLEDState.StaticLEDState.kAutoAlignScoringComplete);
                break;
            case SCORE_RUNNING:
                LED.setArmLEDState(TimedLEDState.BlinkingLEDState.kAutoAimScoring);
                break;
            case STABILIZING_TRAJECTORY:
                LED.setArmLEDState(TimedLEDState.BlinkingLEDState.kAutoAimPreparing);
                break;
            default:
                break;
            
        }
    }

    public void handleScoreAutoAlignLEDs() {
        switch (mAutoAlignState) {
            case DRIVING_TO_TARGET:
                //TODO: distance to target point
                break;
            case SCORE_FINISHED:
                LED.setArmLEDState(TimedLEDState.StaticLEDState.kAutoAlignScoringComplete);
                break;
            case SCORE_RUNNING:
                LED.setArmLEDState(TimedLEDState.BlinkingLEDState.kAutoAlignScoring);
                break;
            default:
                break;
        }
    }

    public void handleClimbingLEDs() {
        switch (mClimbingState) {
            case CLIMBING_DUTY_CYCLE:
                LED.setArmLEDState(TimedLEDState.BlinkingLEDState.kClimbRunningDutyCycle);
                break;
            case CLIMBING_MANUAL:
                LED.setArmLEDState(TimedLEDState.RSLBasedLEDState.kClimbManual);
                break;
            case RAISED:
                LED.setArmLEDState(TimedLEDState.StaticLEDState.kClimbHeightTargetReached);
                break;
            case RAISING:
                LED.setArmLEDState(TimedLEDState.RSLBasedLEDState.kClimbRaising);
                break;
            default:
                break;
        }
    }

    public ObjectiveTracker(Intake intake, Indexer indexer) {
        mIntake = intake;
        mIndexer = indexer;
    }

    @Override
    public void periodic() {
        LED.setArmLEDState(TimedLEDState.StaticLEDState.kStaticOff);
        switch (mMasterObjective) {
            case NONE:
                handleNoneLEDs();
                break;
            case INTAKE_GROUND:

                break;
            case INTAKE_SOURCE_AUTOALIGN:

                break;
            case SCORE_SPEAKER_AUTOAIM:

                break;
            case SCORE_SPEAKER_AUTOALIGN:

                break;
            case SCORE_AMP_AUTOALIGN:

                break;
            default:
                break;
        }
    }
}
