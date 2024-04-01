package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.armTilt.ArmTilt;
import frc.robot.subsystems.armTilt.ArmTilt.ArmPositionPreset;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerStateMachine.IndexerWantedAction;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeStateMachine.IntakeWantedAction;
import frc.robot.subsystems.intakeDeploy.IntakeDeploy;
import frc.robot.subsystems.intakeDeploy.IntakeDeploy.IntakePositionPreset;
import frc.robot.subsystems.objectiveTracker.ObjectiveTracker;
import frc.robot.subsystems.objectiveTracker.ObjectiveTracker.MasterObjective;

public class IntakeAntiHandoff extends SequentialCommandGroup {
    public IntakeAntiHandoff(ArmTilt armTilt, IntakeDeploy intakeDeploy, Intake intake, Indexer indexer, ObjectiveTracker objective) {
        addCommands(
            Commands.runOnce(() -> objective.setMasterObjective(MasterObjective.INTAKE_HANDOFF)),
            armTilt.setPositionBlockingCommand(ArmPositionPreset.STOWED),
            intakeDeploy.setPositionBlockingCommand(IntakePositionPreset.HANDOFF),
            indexer.setActionCommand(IndexerWantedAction.INTAKE),
            intake.setActionCommand(IntakeWantedAction.INTAKE_CONSTANT),
            indexer.waitUntilNoteCommand(),
            indexer.setActionCommand(IndexerWantedAction.OFF),
            intake.setActionCommand(IntakeWantedAction.OFF),
            intakeDeploy.setPositionBlockingCommand(IntakePositionPreset.STOWED)
            .finallyDo(() -> {
                objective.setMasterObjective(MasterObjective.NONE);
            })
        );
    }
}
