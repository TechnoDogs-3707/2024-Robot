package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.Arm.GoalState;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerStateMachine.IndexerWantedAction;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeStateMachine.IntakeWantedAction;
import frc.robot.subsystems.objectiveTracker.ObjectiveTracker;
import frc.robot.subsystems.objectiveTracker.ObjectiveTracker.IntakeGroundState;
import frc.robot.subsystems.objectiveTracker.ObjectiveTracker.MasterObjective;
import frc.robot.subsystems.shooterFlywheels.ShooterFlywheels;
import frc.robot.subsystems.shooterFlywheels.ShooterFlywheelsStateMachine.FlywheelsWantedAction;

public class IntakeNoteGroundToIndexer extends SequentialCommandGroup {
    public IntakeNoteGroundToIndexer(Arm arm, Intake intake, Indexer indexer, ShooterFlywheels flywheels, ObjectiveTracker objective) {
        addCommands(
            arm.setGoalCommand(GoalState.INTAKE_GROUND)
            .alongWith(
                Commands.runOnce(() -> objective.setMasterObjective(MasterObjective.INTAKE_GROUND)),
                Commands.runOnce(() -> objective.setIntakeGroundState(IntakeGroundState.TO_INDEXER)),
                intake.setActionCommand(IntakeWantedAction.INTAKE_CONSTANT),
                indexer.setActionCommand(IndexerWantedAction.INTAKE),
                indexer.waitUntilNoteCommand(),
                flywheels.setActionCommand(FlywheelsWantedAction.IDLE)
            ).finallyDo(() -> {
                arm.setGoalState(GoalState.STOW);
                intake.setWantedAction(IntakeWantedAction.OFF);
                indexer.setWantedAction(IndexerWantedAction.OFF);
                objective.setMasterObjective(MasterObjective.NONE);
            })
        );
    }
}
