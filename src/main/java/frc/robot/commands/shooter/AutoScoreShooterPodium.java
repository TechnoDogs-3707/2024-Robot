package frc.robot.commands.shooter;

import java.util.function.BooleanSupplier;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.flywheels.Flywheels;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.objectiveTracker.ObjectiveTracker;
import frc.robot.subsystems.tilt.ShooterTilt;
import frc.robot.subsystems.tilt.ShooterTilt.TiltGoalState;

public class AutoScoreShooterPodium extends SequentialCommandGroup {
    public AutoScoreShooterPodium(Drive drive, Indexer indexer, ShooterTilt tilt, Flywheels flywheels, ObjectiveTracker objective, BooleanSupplier scoreOverride) {
        addCommands(
            new AutoScoreShooter(drive, indexer, tilt, flywheels, objective, () -> 65.0, () -> 65.0, () -> TiltGoalState.PODIUM, scoreOverride)
        );
    }
}
