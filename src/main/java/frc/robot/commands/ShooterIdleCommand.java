package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.shooterFlywheels.ShooterFlywheels;
import frc.robot.subsystems.shooterFlywheels.ShooterFlywheelsStateMachine.FlywheelsWantedAction;

public class ShooterIdleCommand extends Command {
    private final ShooterFlywheels mFlywheels;
    private final Indexer mIndexer;

    public ShooterIdleCommand(ShooterFlywheels flywheels, Indexer indexer) {
        mFlywheels = flywheels;
        mIndexer = indexer;

        addRequirements(mFlywheels);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        if (mIndexer.temporaryHasGamepiece()) {
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
