package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.Arm.GoalState;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerStateMachine.IndexerWantedAction;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeStateMachine.IntakeWantedAction;
import frc.robot.subsystems.objectiveTracker.ObjectiveTracker;
import frc.robot.subsystems.objectiveTracker.ObjectiveTracker.IntakeGroundState;
import frc.robot.subsystems.objectiveTracker.ObjectiveTracker.MasterObjective;

public class IntakeNoteGroundToIndexer extends SequentialCommandGroup {
    public IntakeNoteGroundToIndexer(Arm arm, Intake intake, Indexer indexer, ObjectiveTracker objective) {
        addCommands(
            arm.setGoalCommand(GoalState.INTAKE_GROUND)
            .alongWith(
                new InstantCommand(() -> objective.setMasterObjective(MasterObjective.INTAKE_GROUND)),
                new InstantCommand(() -> objective.setIntakeGroundState(IntakeGroundState.TO_INDEXER)),
                intake.setActionCommand(IntakeWantedAction.INTAKE_CONSTANT),
                indexer.setActionCommand(IndexerWantedAction.INTAKE),
                indexer.waitUntilNoteCommand()
            ).finallyDo(() -> {
                arm.setGoalState(GoalState.STOW);
                intake.setWantedAction(IntakeWantedAction.OFF);
                indexer.setWantedAction(IndexerWantedAction.OFF);
                objective.setMasterObjective(MasterObjective.NONE);
            })
        );
    }
}
