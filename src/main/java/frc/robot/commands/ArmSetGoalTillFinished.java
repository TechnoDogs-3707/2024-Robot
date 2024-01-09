package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.Arm.GoalState;

public class ArmSetGoalTillFinished extends Command {
    private final Arm arm;
    private final GoalState goalState;

    public ArmSetGoalTillFinished(Arm arm, GoalState goalState) {
        this.arm = arm;
        this.goalState = goalState;

        addRequirements(arm);
    }

    @Override
    public void initialize() {
        arm.setGoalState(goalState);
    }

    @Override
    public void execute() {
        
    }

    @Override
    public void end(boolean interrupted) {
        
    }

    @Override
    public boolean isFinished() {
        return arm.atGoal() || arm.getResetMotionPlanner();
    }
}
