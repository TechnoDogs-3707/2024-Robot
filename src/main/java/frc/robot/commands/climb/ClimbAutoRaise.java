package frc.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.climb.ClimbConstants;
import frc.robot.subsystems.climb.Climb.ClimbMode;
import frc.robot.subsystems.objectiveTracker.ObjectiveTracker;
import frc.robot.subsystems.objectiveTracker.ObjectiveTracker.ClimbingState;
import frc.robot.subsystems.objectiveTracker.ObjectiveTracker.MasterObjective;

public class ClimbAutoRaise extends SequentialCommandGroup {
    public ClimbAutoRaise(Climb climb, ObjectiveTracker objective) {
        addCommands(
            Commands.runOnce(() -> objective.setMasterObjective(MasterObjective.CLIMB)),
            Commands.runOnce(() -> objective.setClimbingState(ClimbingState.RAISING)),
            climb.setModeCommand(ClimbMode.PID_EXTEND),
            Commands.waitUntil(() -> climb.isAtPosition(ClimbConstants.kFullExtensionPosition)),
            Commands.runOnce(() -> objective.setClimbingState(ClimbingState.RAISED))
        );
    }
}
