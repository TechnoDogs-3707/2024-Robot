package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.shooterFlywheels.ShooterFlywheels;
import frc.robot.subsystems.shooterFlywheels.ShooterFlywheelsStateMachine.FlywheelsWantedAction;
import frc.robot.subsystems.shooterTilt.ShooterTilt;
import frc.robot.subsystems.shooterTilt.ShooterTilt.ShooterTiltGoalState;

public class ShooterScoreSubwooferCommand extends Command {
    private final ShooterTilt mTilt;
    private final ShooterFlywheels mFlywheels;
    private final Indexer mIndexer;

    public ShooterScoreSubwooferCommand(ShooterTilt tilt, ShooterFlywheels flywheels, Indexer indexer) {
        mTilt = tilt;
        mFlywheels = flywheels;
        mIndexer = indexer;

        addRequirements(mTilt, mFlywheels);
    }

    @Override
    public void initialize() {
        if (!mIndexer.temporaryHasGamepiece()) { // cancel shoot command if we don't have a gamepiece
            this.cancel();
        } else {
            mFlywheels.setSetpointSpeedLeft(30);
            mFlywheels.setSetpointSpeedRight(100);
            mFlywheels.setWantedAction(FlywheelsWantedAction.SHOOT);
            mTilt.setGoalState(ShooterTiltGoalState.CLOSE);
        }
    }
    
    @Override
    public void execute() {
        
    }

    @Override
    public void end(boolean interrupted) {
        mFlywheels.setWantedAction(FlywheelsWantedAction.OFF);
        mTilt.setGoalState(ShooterTiltGoalState.STOW);
    }
}
