package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotStateTracker;
import frc.robot.lib.drive.AutoAlignPointSelector;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.objectiveTracker.ObjectiveTracker;

public class DriveAutoAlignCommand extends SequentialCommandGroup {
    public DriveAutoAlignCommand(Drive drive, ObjectiveTracker tracker, Supplier<Boolean> ignorePreference) {
        addCommands(
            drive.autoAlignAndWaitCommand(
                () -> AutoAlignPointSelector.getAlignTarget(
                    RobotStateTracker.getInstance().getCurrentRobotPose(), 
                    tracker.getRequestedAlignment(ignorePreference.get())
                )
            ).finallyDo(() -> {
                drive.stop();
            })
        );
    }
}