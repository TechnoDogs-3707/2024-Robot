package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.flywheels.Flywheels;
import frc.robot.subsystems.flywheels.FlywheelsStateMachine.FlywheelsWantedAction;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerStateMachine.IndexerWantedAction;
import frc.robot.subsystems.objectiveTracker.ObjectiveTracker;
import frc.robot.subsystems.objectiveTracker.ObjectiveTracker.AutoAlignScoreState;
import frc.robot.subsystems.objectiveTracker.ObjectiveTracker.MasterObjective;
import frc.robot.subsystems.tilt.Tilt;
import frc.robot.subsystems.tilt.Tilt.TiltGoalState;

public class AutoScoreShooter extends SequentialCommandGroup {
    public AutoScoreShooter(Drive drive, Indexer indexer, Tilt tilt, Flywheels flywheels, ObjectiveTracker objective, Supplier<Double> topFlywheelSpeed, Supplier<Double> bottomFlywheelSpeed, Supplier<TiltGoalState> tiltGoal, BooleanSupplier scoreOverride) {
        addCommands(
            Commands.runOnce(() -> objective.setMasterObjective(MasterObjective.SCORE_SPEAKER_AUTOALIGN))
            .andThen(Commands.runOnce(() -> objective.setAutoAlignState(AutoAlignScoreState.DRIVING_TO_TARGET)))
            .alongWith(
                Commands.runOnce(() -> flywheels.setSetpointSpeedTop(topFlywheelSpeed.get())),
                Commands.runOnce(() -> flywheels.setSetpointSpeedBottom(bottomFlywheelSpeed.get())),
                indexer.setActionCommand(IndexerWantedAction.INTAKE),
                flywheels.setActionCommand(FlywheelsWantedAction.SHOOT),//.andThen(flywheels.waitUntilStateCommand(FlywheelsSystemState.READY)),
                tilt.setGoalCommand(tiltGoal.get()),//.andThen(Commands.waitUntil(tilt::withinTolerance)),
                Commands.waitUntil(drive::autoAlignAtTarget).raceWith(Commands.waitUntil(scoreOverride))
            ).andThen(
                Commands.runOnce(() -> objective.setAutoAlignState(AutoAlignScoreState.SCORE_RUNNING)),
                indexer.setActionCommand(IndexerWantedAction.SCORE),
                indexer.waitUntilNoteGoneCommand(),
                Commands.waitSeconds(0.25)
            ).finallyDo(() -> {
                objective.setMasterObjective(MasterObjective.NONE);
                indexer.setActionCommand(IndexerWantedAction.OFF);
                tilt.setGoalState(TiltGoalState.STOW);
                flywheels.setWantedAction(FlywheelsWantedAction.OFF);
            })
        );
    }
}
