// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.drive.Drive;

/** Add your docs here. */
public class DriveUtilityCommandFactory {
    public static final Command resetGyro(Drive drive) {
        return new InstantCommand(drive::reseedRotation, drive);
    }

    public static final Command reseedPosition(Drive drive) {
        return new InstantCommand(() -> {}, drive);
    }

    public static final Command failGyro(Drive drive) {
        return new InstantCommand(() -> drive.setIgnoreGyro(true), drive);
    }

    public static final Command unFailGyro(Drive drive) {
        return new InstantCommand(() -> drive.setIgnoreGyro(false), drive);
    }

    public static final Command failDriveAssist(Drive drive) {
        return new InstantCommand(() -> drive.setDriveAssistFail(true), drive);
    }

    public static final Command unFailDriveAssist(Drive drive) {
        return new InstantCommand(() -> drive.setDriveAssistFail(false), drive);
    }

    public static final Command enableAutoAlignNote(Drive drive) {
        return new InstantCommand(() -> drive.enableAutoAlignNoteMode());
    }

        public static final Command disableAutoAlignNote(Drive drive) {
        return new InstantCommand(() -> drive.disableAutoAlignNoteMode());
    }
}
