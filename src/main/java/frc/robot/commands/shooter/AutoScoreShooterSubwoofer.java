package frc.robot.commands.shooter;

import java.util.function.BooleanSupplier;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.flywheels.Flywheels;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.objectiveTracker.ObjectiveTracker;
import frc.robot.subsystems.tilt.Tilt;
import frc.robot.subsystems.tilt.Tilt.TiltGoalState;

public class AutoScoreShooterSubwoofer extends SequentialCommandGroup {
    public AutoScoreShooterSubwoofer(Drive drive, Indexer indexer, Tilt tilt, Flywheels flywheels, ObjectiveTracker objective, BooleanSupplier scoreOverride) {
        addCommands(
            new AutoScoreShooter(drive, indexer, tilt, flywheels, objective, () -> 60.0, () -> 60.0, () -> TiltGoalState.CLOSE, scoreOverride)
        );
    }
}
