package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.Arm.GoalState;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerStateMachine.IndexerWantedAction;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeStateMachine.IntakeWantedAction;
import frc.robot.subsystems.objectiveTracker.ObjectiveTracker;
import frc.robot.subsystems.objectiveTracker.ObjectiveTracker.MasterObjective;
import frc.robot.subsystems.shooterFlywheels.ShooterFlywheels;
import frc.robot.subsystems.shooterFlywheels.ShooterFlywheelsStateMachine.FlywheelsWantedAction;

public class IntakeHandoffToIndexer extends SequentialCommandGroup {
    public IntakeHandoffToIndexer(Arm arm, Intake intake, Indexer indexer, ShooterFlywheels flywheels, ObjectiveTracker objective) {
        addCommands(
            arm.setGoalCommand(GoalState.HANDOFF)
            .alongWith(
                Commands.runOnce(() -> objective.setMasterObjective(MasterObjective.INTAKE_HANDOFF))
            )
            .andThen(
                indexer.setActionCommand(IndexerWantedAction.INTAKE).alongWith(
                    intake.setActionCommand(IntakeWantedAction.INTAKE_HANDOFF),
                    flywheels.setActionCommand(FlywheelsWantedAction.IDLE)
                ).andThen(
                    intake.waitUntilEmptyCommand()),
                    arm.setGoalCommand(GoalState.STOW),
                    indexer.waitUntilNoteCommand()
            ).finallyDo(() -> {
                objective.setMasterObjective(MasterObjective.NONE);
                intake.setWantedAction(IntakeWantedAction.OFF);
                indexer.setWantedAction(IndexerWantedAction.OFF);
                arm.setGoalState(GoalState.STOW);
            })
        );
    }
}
