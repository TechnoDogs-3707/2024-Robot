package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.Arm.GoalState;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeStateMachine.IntakeWantedAction;
import frc.robot.subsystems.objectiveTracker.ObjectiveTracker;
import frc.robot.subsystems.objectiveTracker.ObjectiveTracker.AutoAlignIntakeState;
import frc.robot.subsystems.objectiveTracker.ObjectiveTracker.MasterObjective;

public class IntakeNoteSource extends SequentialCommandGroup {
    public IntakeNoteSource(Arm arm, Intake intake, ObjectiveTracker objective) {
        addCommands(
            arm.setGoalCommand(GoalState.INTAKE_SOURCE)
            .alongWith(
                new InstantCommand(() -> objective.setMasterObjective(MasterObjective.INTAKE_SOURCE_AUTOALIGN)),
                new InstantCommand(() -> objective.setAutoIntakeState(AutoAlignIntakeState.DRIVING_TO_TARGET)),
                intake.setActionCommand(IntakeWantedAction.INTAKE_PARTIAL),
                // wait until we are aligned
                new InstantCommand(() -> objective.setAutoIntakeState(AutoAlignIntakeState.ON_TARGET)),
                intake.waitUntilNoteCommand()
            ).finallyDo(() -> {
                objective.setMasterObjective(MasterObjective.NONE);
                intake.setActionCommand(IntakeWantedAction.OFF);
                arm.setGoalState(GoalState.STOW);
            })
        );
    }
}
