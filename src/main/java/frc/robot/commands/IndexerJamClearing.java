package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.Arm.GoalState;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerStateMachine.IndexerWantedAction;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeStateMachine.IntakeWantedAction;

public class IndexerJamClearing extends SequentialCommandGroup {
    public IndexerJamClearing(Arm arm, Intake intake, Indexer indexer) { //TODO: jam clear LED state
        addCommands(
            arm.setGoalCommand(GoalState.STOW).andThen(
                indexer.setActionCommand(IndexerWantedAction.REVERSE),
                intake.setActionCommand(IntakeWantedAction.INTAKE_CONSTANT),
                Commands.idle()
            ).finallyDo(() -> {
                indexer.setWantedAction(IndexerWantedAction.OFF);
                intake.setWantedAction(IntakeWantedAction.OFF);
                arm.setGoalState(GoalState.STOW);
            })
        );
    }
}
