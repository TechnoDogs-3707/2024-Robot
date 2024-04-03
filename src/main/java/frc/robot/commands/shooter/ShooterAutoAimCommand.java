package frc.robot.commands.shooter;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotState;
import frc.robot.lib.drive.SwerveHeadingController;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.flywheels.Flywheels;
import frc.robot.subsystems.flywheels.FlywheelsStateMachine.FlywheelsSystemState;
import frc.robot.subsystems.flywheels.FlywheelsStateMachine.FlywheelsWantedAction;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerStateMachine.IndexerWantedAction;
import frc.robot.subsystems.objectiveTracker.ObjectiveTracker;
import frc.robot.subsystems.objectiveTracker.ObjectiveTracker.MasterObjective;
import frc.robot.subsystems.objectiveTracker.ObjectiveTracker.SpeakerAutoAimState;
import frc.robot.subsystems.tilt.Tilt;
import frc.robot.subsystems.tilt.Tilt.TiltGoalState;

public class ShooterAutoAimCommand extends SequentialCommandGroup {
    private static double kMaxShootingDistance = 4.85;
    public ShooterAutoAimCommand(Drive drive, Indexer indexer, Tilt tilt, Flywheels flywheels, ObjectiveTracker objective, BooleanSupplier dontShoot) {
        addCommands(
            Commands.runOnce(() -> objective.setMasterObjective(MasterObjective.SCORE_SPEAKER_AUTOAIM))
            .andThen(Commands.runOnce(() -> objective.setAutoAimState(SpeakerAutoAimState.PREPARING_ROBOT)))
            .alongWith(
                Commands.runOnce(() -> flywheels.setSetpointSpeedTop(60)),
                Commands.runOnce(() -> flywheels.setSetpointSpeedBottom(60)),
                indexer.setActionCommand(IndexerWantedAction.INTAKE),
                flywheels.setActionCommand(FlywheelsWantedAction.SHOOT),
                tilt.setGoalCommand(TiltGoalState.AUTO_AIM)
            ).andThen(
                flywheels.waitUntilStateCommand(FlywheelsSystemState.READY),
                Commands.waitUntil(tilt::withinTolerance),
                Commands.runOnce(() -> objective.setAutoAimState(SpeakerAutoAimState.WAITING_FOR_POSITION))
            ).andThen(
                Commands.waitUntil(() -> {
                    return tilt.withinTolerance() 
                    && drive.isSlowEnoughForAutoShoot() 
                    && flywheels.getSystemState().equals(FlywheelsSystemState.READY)
                    && SwerveHeadingController.getInstance().isAtGoal()
                    && RobotState.getInstance().getAimingParameters().effectiveDistance() <= kMaxShootingDistance
                    && !dontShoot.getAsBoolean();
                }),
                Commands.runOnce(() -> objective.setAutoAimState(SpeakerAutoAimState.SCORE_RUNNING)),
                indexer.setActionCommand(IndexerWantedAction.SCORE),
                indexer.waitUntilNoteGoneCommand(),
                Commands.waitSeconds(0.25),
                Commands.runOnce(() -> objective.setAutoAimState(SpeakerAutoAimState.SCORE_FINISHED)),
                Commands.idle()
            ).finallyDo(() -> {
                objective.setMasterObjective(MasterObjective.NONE);
                indexer.setActionCommand(IndexerWantedAction.OFF);
                tilt.setGoalState(TiltGoalState.STOW);
                flywheels.setWantedAction(FlywheelsWantedAction.OFF);
            })
        );
    }
}
