package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerStateMachine.IndexerWantedAction;
import frc.robot.subsystems.shooterFlywheels.ShooterFlywheels;
import frc.robot.subsystems.shooterFlywheels.ShooterFlywheelsStateMachine.FlywheelsWantedAction;
import frc.robot.subsystems.shooterTilt.ShooterTilt;
import frc.robot.subsystems.shooterTilt.ShooterTilt.ShooterTiltGoalState;

public class IndexerReset extends SequentialCommandGroup{
    public IndexerReset(Indexer indexer, ShooterTilt tilt, ShooterFlywheels flywheels) {
        addCommands(
            indexer.setActionCommand(IndexerWantedAction.OFF).alongWith(
                tilt.setGoalCommand(ShooterTiltGoalState.STOW),
                flywheels.setActionCommand(FlywheelsWantedAction.OFF)
            )
        );
    }
}
