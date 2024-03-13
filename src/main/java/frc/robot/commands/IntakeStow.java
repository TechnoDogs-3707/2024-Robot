package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.Arm.GoalState;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeStateMachine.IntakeWantedAction;

public class IntakeStow extends SequentialCommandGroup {
    public IntakeStow(Arm arm, Intake intake) {
        addCommands(
            arm.setGoalCommand(GoalState.STOW)
            .alongWith(
                intake.setActionCommand(IntakeWantedAction.OFF)
            )
        );
    }
}
