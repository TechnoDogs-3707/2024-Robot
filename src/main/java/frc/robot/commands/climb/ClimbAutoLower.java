package frc.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.climb.Climb.ClimbMode;
import frc.robot.subsystems.objectiveTracker.ObjectiveTracker;
import frc.robot.subsystems.objectiveTracker.ObjectiveTracker.ClimbingState;
import frc.robot.subsystems.objectiveTracker.ObjectiveTracker.MasterObjective;

public class ClimbAutoLower extends SequentialCommandGroup {
    public ClimbAutoLower(Climb climb, ObjectiveTracker objective) {
        addCommands(
            Commands.runOnce(() -> objective.setMasterObjective(MasterObjective.CLIMB)),
            Commands.runOnce(() -> objective.setClimbingState(ClimbingState.RAISING)),
            climb.setModeCommand(ClimbMode.PID_RETRACT),
            Commands.waitUntil(() -> climb.isAtPosition(Constants.Climb.kFullRetractionPosition)),
            Commands.runOnce(() -> objective.setMasterObjective(MasterObjective.NONE))
        );
    }
}
