package frc.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.climb.Climb.ClimbMode;
import frc.robot.subsystems.objectiveTracker.ObjectiveTracker;
import frc.robot.subsystems.objectiveTracker.ObjectiveTracker.MasterObjective;

public class ClimbReset extends SequentialCommandGroup {
    public ClimbReset(Climb climb, ObjectiveTracker objective) {
        addCommands(
            new ClimbAutoLower(climb, objective)
            .finallyDo(() -> {
                climb.setManualThrottle(0);
                climb.setClimbMode(ClimbMode.MANUAL_THROTTLE);
                objective.setMasterObjective(MasterObjective.NONE);
            })
        );
    }
}
