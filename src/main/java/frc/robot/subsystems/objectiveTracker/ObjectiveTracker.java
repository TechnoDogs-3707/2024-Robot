package frc.robot.subsystems.objectiveTracker;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import frc.robot.lib.drive.AutoAlignPointSelector.RequestedAlignment;
import frc.robot.lib.leds.TimedLEDState;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.leds.LED;
import frc.robot.util.poofsUtils.VirtualSubsystem;

public class ObjectiveTracker extends VirtualSubsystem {
    private final Intake mIntake;
    private final Indexer mIndexer;

    public enum MasterObjective {
        NONE,
        INTAKE_GROUND,
        INTAKE_SOURCE_AUTOALIGN,
        INTAKE_HANDOFF,
        SCORE_SPEAKER_AUTOALIGN,
        SCORE_SPEAKER_AUTOAIM,
        SCORE_AMP_AUTOALIGN,
        CLIMB,
        ARM_SCORE_AMP
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
        PREPARING_ROBOT,
        WAITING_FOR_POSITION,
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

    public enum ArmScoreAmpState {
        PREPARING,
        WAITING,
        SCORING,
        DONE
    }

    private MasterObjective mMasterObjective = MasterObjective.NONE;
    private IntakeGroundState mIntakeGroundState = IntakeGroundState.TO_INDEXER;
    private AutoAlignIntakeState mAutoIntakeState = AutoAlignIntakeState.DRIVING_TO_TARGET;
    private SpeakerAutoAimState mAutoAimState = SpeakerAutoAimState.PREPARING_ROBOT;
    private AutoAlignScoreState mAutoAlignState = AutoAlignScoreState.DRIVING_TO_TARGET;
    private ClimbingState mClimbingState = ClimbingState.RAISING;
    private ArmScoreAmpState mArmScoreAmpState = ArmScoreAmpState.PREPARING;

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

    public void setArmScoreAmpState(ArmScoreAmpState intakeHandoffState) {
        mArmScoreAmpState = intakeHandoffState;
    }

    public ArmScoreAmpState getArmScoreAmpState() {
        return mArmScoreAmpState;
    }

    public void handleNoneLEDs() {
        if (mIntake.hasNote()) {
            LED.setArmLEDState(TimedLEDState.StaticLEDState.kNoteInArm);
        } else if (mIndexer.hasNote()) {
            LED.setArmLEDState(TimedLEDState.StaticLEDState.kNoteInIndexer);
        } else {
            LED.setArmLEDState(TimedLEDState.RSLBasedLEDState.kIndexerEmpty);
        }
    }

    public void handleIntakeGroundLEDs() {
       switch (mIntakeGroundState) {
        case TO_INTAKE:
            LED.setArmLEDState(TimedLEDState.BlinkingLEDState.kIntakingToArm);
            break;
        case TO_INDEXER:
            LED.setArmLEDState(TimedLEDState.BlinkingLEDState.kIntakingToIndexer);
            break;
        default:
            break;
       }
    }

    public void handleIntakeSourceAutoAlignLEDs() {
        switch (mAutoIntakeState) {
            case DRIVING_TO_TARGET:
                // TODO: percent full based on distance
                LED.setArmLEDState(TimedLEDState.RSLBasedLEDState.kAutoAimWaiting);
                break;
            case ON_TARGET:
                LED.setArmLEDState(TimedLEDState.BlinkingLEDState.kIntakeAutoAlignOnTarget);
                break;
            default:
                break;
        }
    }

    public void handleHandoffLEDs() {
        LED.setArmLEDState(TimedLEDState.BlinkingLEDState.kHandoffRunning);
    }

    public void handleArmScoreAmpLEDs() {
        switch (mArmScoreAmpState) {
            case PREPARING:
                LED.setArmLEDState(TimedLEDState.BlinkingLEDState.kAmpScorePreparing);
                break;
            case WAITING:
                LED.setArmLEDState(TimedLEDState.RSLBasedLEDState.kAmpScoreWaiting);
                break;
            case SCORING:
                LED.setArmLEDState(TimedLEDState.BlinkingLEDState.kAmpScoreScoring);
                break;
            case DONE:
                LED.setArmLEDState(TimedLEDState.StaticLEDState.kAmpScoreDone);
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
            case PREPARING_ROBOT:
                LED.setArmLEDState(TimedLEDState.BlinkingLEDState.kAutoAimPreparing);
                break;
            case WAITING_FOR_POSITION:
                LED.setArmLEDState(TimedLEDState.RSLBasedLEDState.kAutoAimWaiting);
            default:
                break;
            
        }
    }

    public void handleScoreAutoAlignLEDs() {
        switch (mAutoAlignState) {
            case DRIVING_TO_TARGET:
                //TODO: distance to target point
                LED.setArmLEDState(TimedLEDState.RSLBasedLEDState.kAutoAimWaiting);
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

        mAutoAlignSourcePreference.addDefaultOption("Any", RequestedAlignment.SOURCE_AUTO);
        mAutoAlignSourcePreference.addOption("Left", RequestedAlignment.SOURCE_LEFT);
        mAutoAlignSourcePreference.addOption("Right", RequestedAlignment.SOURCE_RIGHT);

        mAutoAlignSpeakerPreference.addDefaultOption("Any", RequestedAlignment.SHOOTER_AUTO);
        mAutoAlignSpeakerPreference.addOption("Close", RequestedAlignment.SPEAKER_CLOSE);
        mAutoAlignSpeakerPreference.addOption("Podium", RequestedAlignment.SPEAKER_PODIUM);
    }

    @Override
    public void periodic() {
        LED.setArmLEDState(TimedLEDState.StaticLEDState.kStaticOff);
        switch (mMasterObjective) {
            case NONE:
                handleNoneLEDs();
                break;
            case INTAKE_GROUND:
                handleIntakeGroundLEDs();
                break;
            case INTAKE_SOURCE_AUTOALIGN:
                handleIntakeSourceAutoAlignLEDs();
                break;
            case INTAKE_HANDOFF:
                handleHandoffLEDs();
                break;
            case SCORE_SPEAKER_AUTOAIM:
                handleScoreSpeakerAutoAimLEDs();
                break;
            case SCORE_SPEAKER_AUTOALIGN:
            case SCORE_AMP_AUTOALIGN:
                handleScoreAutoAlignLEDs();
                break;
            case CLIMB:
                handleClimbingLEDs();
                break;
            case ARM_SCORE_AMP:
                handleArmScoreAmpLEDs();
                break;
            default:
                break;
        }
    }

    public enum SimpleObjective {
        SOURCE,
        SPEAKER
    }

    public SimpleObjective getSimpleObjective() {
        if (mIndexer.hasNote()) {
            return SimpleObjective.SPEAKER;
        } else {
            return SimpleObjective.SOURCE;
        }
    }

    public final LoggedDashboardChooser<RequestedAlignment> mAutoAlignSourcePreference = new LoggedDashboardChooser<>("Source Align Preference");
    public final LoggedDashboardChooser<RequestedAlignment> mAutoAlignSpeakerPreference = new LoggedDashboardChooser<>("Speaker Align Preference");

    public RequestedAlignment getRequestedAlignment(boolean ignorePreference) {
        SimpleObjective objective = getSimpleObjective();

        switch (objective) {
            case SOURCE:
                return ignorePreference ? RequestedAlignment.SOURCE_AUTO : mAutoAlignSourcePreference.get();
            case SPEAKER:
                return ignorePreference ? RequestedAlignment.SHOOTER_AUTO : mAutoAlignSpeakerPreference.get();
            default:
                return RequestedAlignment.AUTO;
        }
    }
}
