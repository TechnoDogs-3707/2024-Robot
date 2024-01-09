// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.lib.drive.AutoAlignPointSelector;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drive.Drive;

/** Add your docs here. */
public class DriveAutoAlignCommand extends Command {
    private Drive drive;
    private Arm arm;
    private Optional<Pose2d> targetPoint;

    public DriveAutoAlignCommand(Drive drive, Arm arm) {
        addRequirements(drive);

        this.drive = drive;
        this.arm = arm;
        targetPoint = Optional.empty();
    }

    @Override
    public void initialize() {
        // get target point
        targetPoint = AutoAlignPointSelector.chooseTargetPoint(
            drive.getPose(), 
            arm.getRequestedAlignment()
        );
    }

    @Override
    public void execute() {
        targetPoint.ifPresentOrElse(drive::setSnapYTheta, drive::stop);
    }

    @Override
    public void end(boolean interrupted) {
        drive.stop();
    }
}
