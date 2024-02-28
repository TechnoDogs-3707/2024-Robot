package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerStateMachine.IndexerWantedAction;

public class IndexerForceIntake extends SequentialCommandGroup {
    public IndexerForceIntake(Indexer indexer) {
        addCommands(indexer.setActionCommand(IndexerWantedAction.INTAKE));
    }
}
