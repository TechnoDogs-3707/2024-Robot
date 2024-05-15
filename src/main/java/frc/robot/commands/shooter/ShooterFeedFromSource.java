package frc.robot.commands.shooter;

import java.util.function.BooleanSupplier;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ShooterTilt.ShooterTilt;
import frc.robot.subsystems.ShooterTilt.ShooterTilt.TiltGoalState;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.flywheels.Flywheels;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerStateMachine.IndexerWantedAction;
import frc.robot.subsystems.objectiveTracker.ObjectiveTracker;

public class ShooterFeedFromSource extends SequentialCommandGroup {
    public ShooterFeedFromSource(Drive drive, Indexer indexer, ShooterTilt tilt, Flywheels flywheels, ObjectiveTracker objective, BooleanSupplier scoreOverride) {
        addCommands(
            new AutoScoreShooter(drive, indexer, tilt, flywheels, objective, () -> -30.0, () -> -30.0, () -> TiltGoalState.FEED, scoreOverride),
            indexer.setActionCommand(IndexerWantedAction.INTAKE)
        );
    }   
}
