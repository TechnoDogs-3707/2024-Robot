package frc.robot.commands.shooter;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.armTilt.ArmTilt;
import frc.robot.subsystems.armTilt.ArmTilt.ArmPositionPreset;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.flywheels.Flywheels;
import frc.robot.subsystems.flywheels.FlywheelsStateMachine.FlywheelsWantedAction;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerStateMachine.IndexerWantedAction;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeStateMachine.IntakeWantedAction;
import frc.robot.subsystems.intakeDeploy.IntakeDeploy;
import frc.robot.subsystems.intakeDeploy.IntakeDeploy.IntakePositionPreset;
import frc.robot.subsystems.objectiveTracker.ObjectiveTracker;
import frc.robot.subsystems.objectiveTracker.ObjectiveTracker.AutoAlignScoreState;
import frc.robot.subsystems.objectiveTracker.ObjectiveTracker.MasterObjective;
import frc.robot.subsystems.shooterTilt.ShooterTilt;
import frc.robot.subsystems.shooterTilt.ShooterTilt.TiltGoalState;

public class AutoScoreShooterAmpWithHood extends SequentialCommandGroup {
    public AutoScoreShooterAmpWithHood(Drive drive, ArmTilt arm, IntakeDeploy intakeDeploy, Indexer indexer, Intake intake, ShooterTilt tilt, Flywheels flywheels, ObjectiveTracker objective, BooleanSupplier scoreOverride) {
        addCommands(
            Commands.runOnce(() -> objective.setMasterObjective(MasterObjective.SCORE_SPEAKER_AUTOALIGN))
            .andThen(Commands.runOnce(() -> objective.setAutoAlignState(AutoAlignScoreState.DRIVING_TO_TARGET)))
            .alongWith(
                Commands.runOnce(() -> flywheels.setSetpointSpeedTop(32)),
                Commands.runOnce(() -> flywheels.setSetpointSpeedBottom(32)),
                indexer.setActionCommand(IndexerWantedAction.INTAKE),
                flywheels.setActionCommand(FlywheelsWantedAction.SHOOT),//.andThen(flywheels.waitUntilStateCommand(FlywheelsSystemState.READY)),
                tilt.setGoalCommand(TiltGoalState.AMP),//.andThen(Commands.waitUntil(tilt::withinTolerance)),
                arm.setPositionBlockingCommand(ArmPositionPreset.AMP_ONE).andThen(intakeDeploy.setPositionBlockingCommand(IntakePositionPreset.AMP_ONE)),//, intake.setActionCommand(IntakeWantedAction.INTAKE_CONSTANT)),
                Commands.waitUntil(drive::autoAlignAtTarget).raceWith(Commands.waitUntil(scoreOverride))
            ).andThen(
                Commands.runOnce(() -> objective.setAutoAlignState(AutoAlignScoreState.SCORE_RUNNING)),
                indexer.setActionCommand(IndexerWantedAction.SCORE),
                indexer.waitUntilNoteGoneCommand(),
                // Commands.waitSeconds(0.05),
                arm.setPositionBlockingCommand(ArmPositionPreset.AMP_TWO),
                Commands.waitSeconds(0.25)
            ).finallyDo(() -> {
                objective.setMasterObjective(MasterObjective.NONE);
                indexer.setActionCommand(IndexerWantedAction.OFF);
                tilt.setGoalState(TiltGoalState.STOW);
                flywheels.setWantedAction(FlywheelsWantedAction.OFF);
                arm.setPositionPreset(ArmPositionPreset.STOWED);
                // intake.setWantedAction(IntakeWantedAction.OFF);
                intakeDeploy.setPositionPreset(IntakePositionPreset.STOWED);
            })
        );
    }
}
