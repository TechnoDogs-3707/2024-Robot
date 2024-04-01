package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.flywheels.Flywheels;
import frc.robot.subsystems.flywheels.FlywheelsStateMachine.FlywheelsWantedAction;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.objectiveTracker.ObjectiveTracker;

public class ShooterIdleCommand extends Command {
    private final Flywheels mFlywheels;
    private final Indexer mIndexer;

    public ShooterIdleCommand(Flywheels flywheels, Indexer indexer, ObjectiveTracker objective) {
        mFlywheels = flywheels;
        mIndexer = indexer;

        addRequirements(mFlywheels);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        if (mIndexer.hasNote()) {
            mFlywheels.setWantedAction(FlywheelsWantedAction.IDLE);
        } else {
            mFlywheels.setWantedAction(FlywheelsWantedAction.OFF);
        }
    }
    
    @Override
    public void end(boolean interrupted) {
        mFlywheels.setWantedAction(FlywheelsWantedAction.OFF);
    }
}
