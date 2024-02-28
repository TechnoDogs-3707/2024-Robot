package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.objectiveTracker.ObjectiveTracker;
import frc.robot.subsystems.shooterFlywheels.ShooterFlywheels;
import frc.robot.subsystems.shooterTilt.ShooterTilt;
import frc.robot.subsystems.shooterTilt.ShooterTilt.ShooterTiltGoalState;

public class AutoScoreSpeakerSubwoofer extends SequentialCommandGroup {
    public AutoScoreSpeakerSubwoofer(Drive drive, Indexer indexer, ShooterTilt tilt, ShooterFlywheels flywheels, ObjectiveTracker objective, Supplier<Boolean> scoreOverride) {
        addCommands(
            new AutoScoreSpeaker(drive, indexer, tilt, flywheels, objective, () -> ShooterTiltGoalState.CLOSE, scoreOverride)
        );
    }
}
