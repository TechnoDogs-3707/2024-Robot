package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.flywheels.Flywheels;
import frc.robot.subsystems.flywheels.FlywheelsStateMachine.FlywheelsWantedAction;
import frc.robot.subsystems.indexer.Indexer;

public class ShooterPrepare extends SequentialCommandGroup {
    public ShooterPrepare(Indexer indexer, Flywheels flywheels) {
        addCommands(
            Commands.either(
                flywheels.setActionCommand(FlywheelsWantedAction.IDLE), 
                flywheels.setActionCommand(FlywheelsWantedAction.OFF), 
                indexer::hasNote
            ).andThen(Commands.idle())
        );
    }
}
