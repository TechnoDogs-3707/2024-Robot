package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.lib.ScoringIntentTracker;
import frc.robot.lib.ScoringIntentTracker.ScoringIntent;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmIntakeStateMachine;
import frc.robot.subsystems.arm.Arm.GoalState;
import frc.robot.subsystems.arm.ArmIntakeStateMachine.ArmIntakeSystemState;
import frc.robot.subsystems.arm.ArmIntakeStateMachine.ArmIntakeWantedAction;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerStateMachine;
import frc.robot.subsystems.indexer.IndexerStateMachine.IndexerSystemState;
import frc.robot.subsystems.indexer.IndexerStateMachine.IndexerWantedAction;
import frc.robot.subsystems.shooterFlywheels.ShooterFlywheels;
import frc.robot.subsystems.shooterFlywheels.ShooterFlywheelsStateMachine;
import frc.robot.subsystems.shooterFlywheels.ShooterFlywheelsStateMachine.FlywheelsSystemState;
import frc.robot.subsystems.shooterFlywheels.ShooterFlywheelsStateMachine.FlywheelsWantedAction;
import frc.robot.subsystems.shooterTilt.ShooterTilt;
import frc.robot.subsystems.shooterTilt.ShooterTilt.ShooterTiltGoalState;

public class ScoringCommands {
    public static final BooleanSupplier armIsAtGoalState(Arm arm, GoalState goal) {
        return () -> arm.getMeasuredState().isInRange(goal.state);
    }

    public static final BooleanSupplier intakeIsSystemState(Arm arm, ArmIntakeSystemState state) {
        return () -> arm.getIntakeSystemState().equals(state);
    }

    public static final BooleanSupplier indexerIsSystemState(Indexer indexer, IndexerSystemState state) {
        return () -> indexer.getSystemState().equals(state);
    }

    public static final BooleanSupplier shooterIsAtGoalState(ShooterTilt tilt, ShooterTiltGoalState goal) {
        return () -> tilt.getGoalState().equals(goal);
    }

    public static final BooleanSupplier flywheelIsSystemState(ShooterFlywheels flywheels, FlywheelsSystemState state) {
        return () -> flywheels.getSystemState().equals(state);
    }

    public static final Command setArmGoalState(Arm arm, GoalState goalState) {
        return new InstantCommand(() -> arm.setGoalState(goalState), arm);
    }

    public static final Command setArmIntakeAction(Arm arm, ArmIntakeWantedAction wantedAction) {
        return new InstantCommand(() -> arm.setWantedAction(wantedAction), arm);
    }

    public static final Command setIndexerAction(Indexer indexer, IndexerWantedAction wantedAction) {
        return new InstantCommand(() -> indexer.setWantedAction(wantedAction), indexer);
    }

    public static final Command setShooterWheelsAction(ShooterFlywheels flywheels, FlywheelsWantedAction wantedAction) {
        return new InstantCommand(() -> flywheels.setWantedAction(wantedAction), flywheels);
    }

    public static final Command setShooterTiltGoalState(ShooterTilt tilt, ShooterTiltGoalState goalState) {
        return new InstantCommand(() -> tilt.setGoalState(goalState), tilt);
    }

    public static final Command setScoringIntent(ScoringIntent intent) {
        return new InstantCommand(() -> ScoringIntentTracker.setScoringIntent(intent));
    }

    public static final BooleanSupplier scoringIntentIs(ScoringIntent intent) {
        return () -> ScoringIntentTracker.getScoringIntent().equals(intent);
    }

    public static Command stowArm(Arm arm) {
        return new SequentialCommandGroup(
            setArmGoalState(arm, GoalState.STOW),
            new ConditionalCommand(
                setArmIntakeAction(arm, ArmIntakeWantedAction.INTAKE_PARTIAL), 
                setArmIntakeAction(arm, ArmIntakeWantedAction.OFF), 
                arm::intakeHasGamepiece)
        );
    }

    public static Command resetIndexer(Indexer indexer, ShooterTilt tilt, ShooterFlywheels flywheels) {
        return new SequentialCommandGroup(
            new ConditionalCommand(
                new SequentialCommandGroup(
                    setIndexerAction(indexer, IndexerWantedAction.REVERSE),
                    new WaitCommand(0.25),
                    setIndexerAction(indexer, IndexerWantedAction.INTAKE),
                    setShooterWheelsAction(flywheels, FlywheelsWantedAction.IDLE)
                ), 
                new SequentialCommandGroup(
                    setIndexerAction(indexer, IndexerWantedAction.OFF),
                    setShooterWheelsAction(flywheels, FlywheelsWantedAction.OFF)
                ), 
                indexer::temporaryHasGamepiece),
            setShooterTiltGoalState(tilt, ShooterTiltGoalState.STOW)
        );
    }

    public static Command sensorIntakeGroundToIndexer(Arm arm, Indexer indexer, ShooterTilt tilt, ShooterFlywheels flywheels) {
        return new SequentialCommandGroup(
            new ConditionalCommand(
                new SequentialCommandGroup(
                    setArmGoalState(arm, GoalState.STOW),
                    setIndexerAction(indexer, IndexerWantedAction.INTAKE)
                ),
                new SequentialCommandGroup(
                    setShooterWheelsAction(flywheels, FlywheelsWantedAction.OFF),
                    setShooterTiltGoalState(tilt, ShooterTiltGoalState.STOW),
                    setIndexerAction(indexer, IndexerWantedAction.INTAKE),
                    setArmGoalState(arm, GoalState.INTAKE_GROUND),
                    setArmIntakeAction(arm, ArmIntakeWantedAction.INTAKE_HANDOFF),
                    new WaitUntilCommand(indexer::temporaryHasGamepiece)
                ), 
                indexer::temporaryHasGamepiece
            ),
            setArmIntakeAction(arm, ArmIntakeWantedAction.OFF),
            setArmGoalState(arm, GoalState.STOW),
            setShooterWheelsAction(flywheels, FlywheelsWantedAction.IDLE)
        );
    }

    public static Command sensorIntakeGroundToHold(Arm arm, Indexer indexer, ShooterTilt tilt, ShooterFlywheels flywheels) {
        return new SequentialCommandGroup(
            new ConditionalCommand(
                new SequentialCommandGroup(
                    // hand off gamepiece to intake
                ), 
                new SequentialCommandGroup(
                    new ConditionalCommand(
                        new SequentialCommandGroup(
                            
                        ), 
                        new SequentialCommandGroup(
                            setArmIntakeAction(arm, ArmIntakeWantedAction.INTAKE_PARTIAL),
                            setArmGoalState(arm, GoalState.INTAKE_GROUND),
                            new WaitUntilCommand(intakeIsSystemState(arm, ArmIntakeSystemState.INTAKE_PARTIAL_FULL))
                        ), 
                        arm::intakeHasGamepiece),
                        setArmIntakeAction(arm, ArmIntakeWantedAction.OFF),
                        setArmGoalState(arm, GoalState.STOW)
                ), 
                indexer::temporaryHasGamepiece)
        );
    }

    public static Command scoreSpeakerClose(Indexer indexer, ShooterTilt tilt, ShooterFlywheels flywheels) {
        return new SequentialCommandGroup(
            new ConditionalCommand(
                new SequentialCommandGroup(
                    setIndexerAction(indexer, IndexerWantedAction.INTAKE),
                    setShooterWheelsAction(flywheels, FlywheelsWantedAction.SHOOT),
                    setShooterTiltGoalState(tilt, ShooterTiltGoalState.CLOSE)
                ), 
                new SequentialCommandGroup(
                    
                ), 
                indexer::temporaryHasGamepiece
            )
        );
    }

    public static Command runAmpScorer(Arm arm) {
        return new SequentialCommandGroup(
            setArmIntakeAction(arm, ArmIntakeWantedAction.INTAKE_CONSTANT)
        );
    }

    public static Command armSetAmp(Arm arm) {
        return new SequentialCommandGroup(
            setArmIntakeAction(arm, ArmIntakeWantedAction.OFF),
            setArmGoalState(arm, GoalState.SCORE_AMP)
        );
    }
}
