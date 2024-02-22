package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.shooterFlywheels.ShooterFlywheels;
import frc.robot.subsystems.shooterFlywheels.ShooterFlywheelsStateMachine.FlywheelsWantedAction;

public class ShooterAutomaticCommand extends Command {
    private final ShooterFlywheels mFlywheels;
    private final Indexer mIndexer;

    private final Timer mEmptyTimer;

    public ShooterAutomaticCommand(ShooterFlywheels flywheels, Indexer indexer) {
        mFlywheels = flywheels;
        mIndexer = indexer;

        mEmptyTimer = new Timer();

        addRequirements(mFlywheels);
    }

    @Override
    public void initialize() {
        mEmptyTimer.restart();
    }

    @Override
    public void execute() {
        if (mIndexer.temporaryHasGamepiece()) {
            mEmptyTimer.stop();
            mEmptyTimer.reset();
            mFlywheels.setWantedAction(FlywheelsWantedAction.IDLE);
        } else {
            mEmptyTimer.start();
            if (mEmptyTimer.hasElapsed(5)) {
                mFlywheels.setWantedAction(FlywheelsWantedAction.OFF);
            } else {
                mFlywheels.setWantedAction(FlywheelsWantedAction.IDLE);
            }
        }
    }
    
    @Override
    public void end(boolean interrupted) {
        mFlywheels.setWantedAction(FlywheelsWantedAction.OFF);
    }
}
