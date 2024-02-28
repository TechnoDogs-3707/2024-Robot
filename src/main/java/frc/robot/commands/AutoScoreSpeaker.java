package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotStateTracker;
import frc.robot.lib.drive.AutoAlignPointSelector;
import frc.robot.lib.drive.AutoAlignPointSelector.RequestedAlignment;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerStateMachine.IndexerWantedAction;
import frc.robot.subsystems.objectiveTracker.ObjectiveTracker;
import frc.robot.subsystems.objectiveTracker.ObjectiveTracker.AutoAlignScoreState;
import frc.robot.subsystems.objectiveTracker.ObjectiveTracker.MasterObjective;
import frc.robot.subsystems.shooterFlywheels.ShooterFlywheels;
import frc.robot.subsystems.shooterFlywheels.ShooterFlywheelsStateMachine.FlywheelsWantedAction;
import frc.robot.subsystems.shooterTilt.ShooterTilt;
import frc.robot.subsystems.shooterTilt.ShooterTilt.ShooterTiltGoalState;

public class AutoScoreSpeaker extends SequentialCommandGroup {
    public AutoScoreSpeaker(Drive drive, Indexer indexer, ShooterTilt tilt, ShooterFlywheels flywheels, ObjectiveTracker objective, Supplier<ShooterTiltGoalState> tiltGoal, Supplier<Boolean> scoreOverride) {
        addCommands(
            Commands.waitUntil(drive::autoAlignAtTarget)
            .raceWith(Commands.waitUntil(scoreOverride::get))
            .alongWith(
                Commands.runOnce(() -> objective.setMasterObjective(MasterObjective.SCORE_SPEAKER_AUTOALIGN))
                .andThen(Commands.runOnce(() -> objective.setAutoAlignState(AutoAlignScoreState.DRIVING_TO_TARGET))),
                Commands.runOnce(() -> flywheels.setSetpointSpeedLeft(35)),
                Commands.runOnce(() -> flywheels.setSetpointSpeedRight(100)),
                flywheels.setActionCommand(FlywheelsWantedAction.SHOOT),
                tilt.setGoalCommand(tiltGoal.get())
            ).andThen(
                Commands.runOnce(() -> objective.setAutoAlignState(AutoAlignScoreState.SCORE_RUNNING)),
                indexer.setActionCommand(IndexerWantedAction.SCORE),
                indexer.waitUntilNoteGoneCommand()
            ).finallyDo(() -> {
                objective.setMasterObjective(MasterObjective.NONE);
                indexer.setActionCommand(IndexerWantedAction.OFF);
                tilt.setGoalState(ShooterTiltGoalState.STOW);
                flywheels.setWantedAction(FlywheelsWantedAction.OFF);
            })
        );
    }
}
