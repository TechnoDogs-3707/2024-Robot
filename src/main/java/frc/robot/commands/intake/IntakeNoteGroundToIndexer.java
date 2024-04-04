package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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

public class IntakeNoteGroundToIndexer extends SequentialCommandGroup {
    public IntakeNoteGroundToIndexer(IntakeDeploy intakeDeploy, Intake intake, Indexer indexer, Flywheels flywheels, ObjectiveTracker objective) {
        addCommands(
            intakeDeploy.setPositionBlockingCommand(IntakePositionPreset.DEPLOYED)
            .alongWith(
                Commands.runOnce(() -> objective.setMasterObjective(MasterObjective.INTAKE_GROUND)),
                Commands.runOnce(() -> objective.setIntakeGroundState(IntakeGroundState.TO_INDEXER)),
                intake.setActionCommand(IntakeWantedAction.INTAKE_CONSTANT),
                indexer.setActionCommand(IndexerWantedAction.INTAKE),
                indexer.waitUntilNoteCommand(),
                flywheels.setActionCommand(FlywheelsWantedAction.IDLE)
            ).finallyDo(() -> {
                intakeDeploy.setPositionPreset(IntakePositionPreset.STOWED);
                intake.setWantedAction(IntakeWantedAction.OFF);
                if (indexer.hasNote()) {
                    indexer.setWantedAction(IndexerWantedAction.INTAKE);
                } else {
                    indexer.setWantedAction(IndexerWantedAction.OFF);
                }
                objective.setMasterObjective(MasterObjective.NONE);
            })
        );
    }
}
