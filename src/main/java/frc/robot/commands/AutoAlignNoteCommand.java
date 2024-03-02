package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;

public class AutoAlignNoteCommand extends Command {
    private final Drive drive;
    public AutoAlignNoteCommand(Drive drive) {
        this.drive = drive;
        addRequirements(this.drive);
    }

    @Override
    public void initialize() {
        drive.stop();
        drive.setAutAlignNoteMode();
    }

    @Override
    public void end(boolean interrupted) {
        // drive.setXMode(false);
    }
}
