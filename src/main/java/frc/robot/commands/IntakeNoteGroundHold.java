package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.Arm.GoalState;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeStateMachine.IntakeWantedAction;
import frc.robot.subsystems.objectiveTracker.ObjectiveTracker;
import frc.robot.subsystems.objectiveTracker.ObjectiveTracker.IntakeGroundState;
import frc.robot.subsystems.objectiveTracker.ObjectiveTracker.MasterObjective;

public class IntakeNoteGroundHold extends SequentialCommandGroup {
    public IntakeNoteGroundHold(Arm arm, Intake intake, ObjectiveTracker objective) {
        addCommands(
            arm.setGoalCommand(GoalState.INTAKE_GROUND).alongWith(
                new InstantCommand(() -> objective.setMasterObjective(MasterObjective.INTAKE_GROUND)),
                new InstantCommand(() -> objective.setIntakeGroundState(IntakeGroundState.TO_INTAKE)),
                intake.setActionCommand(IntakeWantedAction.INTAKE_PARTIAL)
            ).andThen(
                intake.waitUntilNoteCommand()
            ).andThen(
                arm.setGoalCommand(GoalState.STOW)
            ).finallyDo(() -> {
                arm.setGoalState(GoalState.STOW);
                intake.setActionCommand(IntakeWantedAction.OFF);
                objective.setMasterObjective(MasterObjective.NONE);
            })
            
        );
    }
}
