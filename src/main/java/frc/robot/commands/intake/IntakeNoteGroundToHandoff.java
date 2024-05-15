package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.armTilt.ArmTilt;
import frc.robot.subsystems.armTilt.ArmTilt.ArmPositionPreset;
import frc.robot.subsystems.flywheels.Flywheels;
import frc.robot.subsystems.flywheels.FlywheelsStateMachine.FlywheelsWantedAction;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerStateMachine.IndexerWantedAction;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeStateMachine.IntakeWantedAction;
import frc.robot.subsystems.intakeDeploy.IntakeDeploy;
import frc.robot.subsystems.intakeDeploy.IntakeDeploy.IntakePositionPreset;
import frc.robot.subsystems.objectiveTracker.ObjectiveTracker;
import frc.robot.subsystems.objectiveTracker.ObjectiveTracker.IntakeGroundState;
import frc.robot.subsystems.objectiveTracker.ObjectiveTracker.MasterObjective;
import frc.robot.subsystems.tilt.ShooterTilt;
import frc.robot.subsystems.tilt.ShooterTilt.TiltGoalState;

public class IntakeNoteGroundToHandoff extends SequentialCommandGroup {
    public IntakeNoteGroundToHandoff(ArmTilt armTilt, IntakeDeploy intakeDeploy, Intake intake, Indexer indexer, ShooterTilt tilt, Flywheels flywheels, ObjectiveTracker objective) {
        addCommands(
            flywheels.setActionCommand(FlywheelsWantedAction.OFF),
            tilt.setGoalCommand(TiltGoalState.STOW),
            Commands.runOnce(() -> objective.setMasterObjective(MasterObjective.INTAKE_GROUND)),
            Commands.runOnce(() -> objective.setIntakeGroundState(IntakeGroundState.TO_INTAKE)),
            armTilt.setPositionBlockingCommand(ArmPositionPreset.STOWED),
            intake.setActionCommand(IntakeWantedAction.INTAKE_CONSTANT),
            indexer.setActionCommand(IndexerWantedAction.INTAKE),
            intakeDeploy.setPositionBlockingCommand(IntakePositionPreset.DEPLOYED),
            indexer.waitUntilNoteCommand(),
            indexer.setActionCommand(IndexerWantedAction.OFF),
            intake.setActionCommand(IntakeWantedAction.OFF),
            intakeDeploy.setPositionBlockingCommand(IntakePositionPreset.HANDOFF),
            indexer.setActionCommand(IndexerWantedAction.REVERSE),
            intake.setActionCommand(IntakeWantedAction.REVERSE),
            Commands.waitSeconds(0.5),
            intake.setActionCommand(IntakeWantedAction.OFF),
            indexer.setActionCommand(IndexerWantedAction.OFF),
            intakeDeploy.setPositionBlockingCommand(IntakePositionPreset.STOWED)
            .finallyDo(() -> {
                intake.setWantedAction(IntakeWantedAction.OFF);
                indexer.setWantedAction(IndexerWantedAction.OFF);
                objective.setMasterObjective(MasterObjective.NONE);
            })
        );
    }
}
