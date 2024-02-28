package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.shooterFlywheels.ShooterFlywheels;
import frc.robot.subsystems.shooterFlywheels.ShooterFlywheelsStateMachine.FlywheelsWantedAction;

public class ShooterPrepare extends SequentialCommandGroup {
    public ShooterPrepare(Indexer indexer, ShooterFlywheels flywheels) {
        addCommands(
            Commands.either(
                flywheels.setActionCommand(FlywheelsWantedAction.IDLE), 
                flywheels.setActionCommand(FlywheelsWantedAction.OFF), 
                indexer::hasNote
            ).andThen(Commands.idle())
        );
    }
}
