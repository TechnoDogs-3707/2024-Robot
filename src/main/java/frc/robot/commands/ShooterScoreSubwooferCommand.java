package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooterFlywheels.ShooterFlywheels;
import frc.robot.subsystems.shooterFlywheels.ShooterFlywheelsStateMachine.FlywheelsWantedAction;
import frc.robot.subsystems.shooterTilt.ShooterTilt;
import frc.robot.subsystems.shooterTilt.ShooterTilt.ShooterTiltGoalState;

public class ShooterScoreSubwooferCommand extends Command {
    private final ShooterTilt mTilt;
    private final ShooterFlywheels mFlywheels;

    public ShooterScoreSubwooferCommand(ShooterTilt tilt, ShooterFlywheels flywheels) {
        mTilt = tilt;
        mFlywheels = flywheels;

        addRequirements(mTilt, mFlywheels);
    }

    @Override
    public void initialize() {
        mFlywheels.setSetpointSpeedLeft(35);
        mFlywheels.setSetpointSpeedRight(90);
        mFlywheels.setWantedAction(FlywheelsWantedAction.SHOOT);
        mTilt.setGoalState(ShooterTiltGoalState.CLOSE);
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
