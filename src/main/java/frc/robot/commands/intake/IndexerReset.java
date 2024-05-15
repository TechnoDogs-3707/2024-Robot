package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ShooterTilt.ShooterTilt;
import frc.robot.subsystems.ShooterTilt.ShooterTilt.TiltGoalState;
import frc.robot.subsystems.flywheels.Flywheels;
import frc.robot.subsystems.flywheels.FlywheelsStateMachine.FlywheelsWantedAction;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerStateMachine.IndexerWantedAction;

public class IndexerReset extends SequentialCommandGroup{
    public IndexerReset(Indexer indexer, ShooterTilt tilt, Flywheels flywheels) {
        addCommands(
            indexer.setActionCommand(IndexerWantedAction.OFF).alongWith(
                tilt.setGoalCommand(TiltGoalState.STOW),
                flywheels.setActionCommand(FlywheelsWantedAction.OFF)
            )
        );
    }
}
