package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.Arm.GoalState;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeStateMachine.IntakeWantedAction;
import frc.robot.subsystems.objectiveTracker.ObjectiveTracker;
import frc.robot.subsystems.objectiveTracker.ObjectiveTracker.AutoAlignIntakeState;
import frc.robot.subsystems.objectiveTracker.ObjectiveTracker.MasterObjective;

public class IntakeNoteSource extends SequentialCommandGroup {
    public IntakeNoteSource(Drive drive, Arm arm, Intake intake, ObjectiveTracker objective) {
        addCommands(
            arm.setGoalCommand(GoalState.INTAKE_SOURCE)
            .alongWith(
                Commands.runOnce(() -> objective.setMasterObjective(MasterObjective.INTAKE_SOURCE_AUTOALIGN)),
                intake.setActionCommand(IntakeWantedAction.INTAKE_PARTIAL),
                Commands.runOnce(() -> objective.setAutoIntakeState(AutoAlignIntakeState.DRIVING_TO_TARGET)),
                intake.waitUntilNoteCommand()
            )
            .deadlineWith(
                Commands.waitUntil(drive::autoAlignAtTarget)
                .andThen(() -> objective.setAutoIntakeState(AutoAlignIntakeState.ON_TARGET))
            )
            .finallyDo(() -> {
                objective.setMasterObjective(MasterObjective.NONE);
                intake.setActionCommand(IntakeWantedAction.OFF);
                arm.setGoalState(GoalState.STOW);
            })
        );
    }
}
