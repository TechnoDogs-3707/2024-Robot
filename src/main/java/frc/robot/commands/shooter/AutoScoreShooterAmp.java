package frc.robot.commands.shooter;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ShooterTilt.ShooterTilt;
import frc.robot.subsystems.ShooterTilt.ShooterTilt.TiltGoalState;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.flywheels.Flywheels;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.objectiveTracker.ObjectiveTracker;

public class AutoScoreShooterAmp extends SequentialCommandGroup{
    public AutoScoreShooterAmp(Drive drive, Indexer indexer, ShooterTilt tilt, Flywheels flywheels, ObjectiveTracker objective, BooleanSupplier scoreOverride) {
        addCommands(
            new AutoScoreShooter(drive, indexer, tilt, flywheels, objective, () -> 20.0, () -> 20.0, () -> TiltGoalState.AMP, scoreOverride)
        );
    }
}
