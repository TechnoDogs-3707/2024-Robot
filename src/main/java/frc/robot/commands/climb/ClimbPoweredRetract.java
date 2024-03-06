package frc.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.climb.Climb.ClimbMode;
import frc.robot.subsystems.objectiveTracker.ObjectiveTracker;
import frc.robot.subsystems.objectiveTracker.ObjectiveTracker.ClimbingState;
import frc.robot.subsystems.objectiveTracker.ObjectiveTracker.MasterObjective;

public class ClimbPoweredRetract extends SequentialCommandGroup {
    public ClimbPoweredRetract(Climb climb, ObjectiveTracker objective) {
        addCommands(
            Commands.runOnce(() -> objective.setMasterObjective(MasterObjective.CLIMB)),
            Commands.runOnce(() -> objective.setClimbingState(ClimbingState.CLIMBING_DUTY_CYCLE)),
            climb.setModeCommand(ClimbMode.CLIMB_RETRACT),
            Commands.idle(climb)
        );
    }
}
