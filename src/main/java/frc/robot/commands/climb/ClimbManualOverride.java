package frc.robot.commands.climb;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.objectiveTracker.ObjectiveTracker;
import frc.robot.subsystems.objectiveTracker.ObjectiveTracker.ClimbingState;
import frc.robot.subsystems.objectiveTracker.ObjectiveTracker.MasterObjective;

public class ClimbManualOverride extends SequentialCommandGroup {
    public ClimbManualOverride(Climb climb, ObjectiveTracker objective, Supplier<Double> throttleSupplier) {
        addCommands(
            Commands.runOnce(() -> objective.setMasterObjective(MasterObjective.CLIMB)),
            Commands.runOnce(() -> objective.setClimbingState(ClimbingState.CLIMBING_MANUAL)),
            climb.setThrottleCommand(throttleSupplier)
        );
    }
}
