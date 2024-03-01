package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.flywheels.Flywheels;
import frc.robot.subsystems.flywheels.FlywheelsStateMachine.FlywheelsWantedAction;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerStateMachine.IndexerWantedAction;
import frc.robot.subsystems.tilt.Tilt;
import frc.robot.subsystems.tilt.Tilt.TiltGoalState;

public class IndexerReset extends SequentialCommandGroup{
    public IndexerReset(Indexer indexer, Tilt tilt, Flywheels flywheels) {
        addCommands(
            indexer.setActionCommand(IndexerWantedAction.OFF).alongWith(
                tilt.setGoalCommand(TiltGoalState.STOW),
                flywheels.setActionCommand(FlywheelsWantedAction.OFF)
            )
        );
    }
}
