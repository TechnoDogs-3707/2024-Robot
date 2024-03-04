package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.flywheels.Flywheels;
import frc.robot.subsystems.flywheels.FlywheelsStateMachine.FlywheelsWantedAction;
import frc.robot.subsystems.tilt.Tilt;
import frc.robot.subsystems.tilt.Tilt.TiltGoalState;

public class ShooterScorePodiumCommand extends Command {
    private final Tilt mTilt;
    private final Flywheels mFlywheels;

    public ShooterScorePodiumCommand(Tilt tilt, Flywheels flywheels) {
        mTilt = tilt;
        mFlywheels = flywheels;

        addRequirements(mTilt, mFlywheels);
    }

    @Override
    public void initialize() {
        mFlywheels.setSetpointSpeedTop(35);
        mFlywheels.setSetpointSpeedBottom(100);
        mFlywheels.setWantedAction(FlywheelsWantedAction.SHOOT);
        mTilt.setGoalState(TiltGoalState.PODIUM);
    }
    
    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
        mFlywheels.setWantedAction(FlywheelsWantedAction.OFF);
        mTilt.setGoalState(TiltGoalState.STOW);
    }
}
