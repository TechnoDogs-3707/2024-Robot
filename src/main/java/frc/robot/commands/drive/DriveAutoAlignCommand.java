package frc.robot.commands.drive;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotState;
import frc.robot.lib.drive.AutoAlignPointSelector;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.objectiveTracker.ObjectiveTracker;

public class DriveAutoAlignCommand extends SequentialCommandGroup {
    public DriveAutoAlignCommand(Drive drive, ObjectiveTracker tracker, Supplier<Boolean> ignorePreference) {
        addCommands(
            drive.autoAlignAndWaitCommand(
                () -> AutoAlignPointSelector.getAlignTarget(
                    RobotState.getInstance().getEstimatedPose(), 
                    tracker.getRequestedAlignment(ignorePreference.get())
                )
            ).finallyDo(() -> {
                drive.stop();
            })
        );
    }
}